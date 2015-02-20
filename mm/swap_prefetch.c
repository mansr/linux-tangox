/*
 * linux/mm/swap_prefetch.c
 *
 * Copyright (C) 2005-2007 Con Kolivas
 *
 * Written by Con Kolivas <kernel@kolivas.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/swap.h>
#include <linux/swap-prefetch.h>
#include <linux/ioprio.h>
#include <linux/kthread.h>
#include <linux/pagemap.h>
#include <linux/syscalls.h>
#include <linux/writeback.h>
#include <linux/vmstat.h>
#include <linux/freezer.h>

/*
 * sysctls:
 * swap_prefetch:	0. Disable swap prefetching
 *			1. Prefetch only when idle and not with laptop_mode
 *			2. Prefetch when idle and with laptop_mode
 *			3. Prefetch at all times.
 * swap_prefetch_delay:	Number of seconds to delay prefetching when system
 *			is not idle.
 * swap_prefetch_sleep:	Number of seconds to put kprefetchd to sleep when
 *			unable to prefetch.
 */
int swap_prefetch __read_mostly = 1;
int swap_prefetch_delay __read_mostly = 1;
int swap_prefetch_sleep __read_mostly = 5;

#define PREFETCH_DELAY		(HZ * swap_prefetch_delay)
#define PREFETCH_SLEEP		((HZ * swap_prefetch_sleep) ? : 1)

struct swapped_root {
	unsigned long		busy;		/* vm busy */
	spinlock_t		lock;		/* protects all data */
	struct list_head	list;		/* MRU list of swapped pages */
	struct radix_tree_root	swap_tree;	/* Lookup tree of pages */
	unsigned int		count;		/* Number of entries */
	unsigned int		maxcount;	/* Maximum entries allowed */
	struct kmem_cache	*cache;		/* Of struct swapped_entry */
};

static struct swapped_root swapped = {
	.lock		= SPIN_LOCK_UNLOCKED,
	.list  		= LIST_HEAD_INIT(swapped.list),
	.swap_tree	= RADIX_TREE_INIT(GFP_ATOMIC),
};

static struct task_struct *kprefetchd_task;

/*
 * We check to see no part of the vm is busy. If it is this will interrupt
 * trickle_swap and wait another PREFETCH_DELAY. Purposefully racy.
 */
inline void delay_swap_prefetch(void)
{
	if (!test_bit(0, &swapped.busy))
		__set_bit(0, &swapped.busy);
}

/*
 * If laptop_mode is enabled don't prefetch to avoid hard drives
 * doing unnecessary spin-ups unless swap_prefetch is explicitly
 * set to a higher value.
 */
static inline int prefetch_enabled(void)
{
	if (swap_prefetch <= laptop_mode)
		return 0;
	return 1;
}

static int kprefetchd_awake;

/*
 * Drop behind accounting which keeps a list of the most recently used swap
 * entries. Entries are removed lazily by kprefetchd.
 */
void add_to_swapped_list(struct page *page)
{
	struct swapped_entry *entry;
	unsigned long index, flags;

	if (!prefetch_enabled())
		goto out;

	spin_lock_irqsave(&swapped.lock, flags);
	if (swapped.count >= swapped.maxcount) {
		/*
		 * Once the number of entries exceeds maxcount we start
		 * removing the least recently used entries.
		 */
		entry = list_entry(swapped.list.next,
			struct swapped_entry, swapped_list);
		radix_tree_delete(&swapped.swap_tree, entry->swp_entry.val);
		list_del(&entry->swapped_list);
		swapped.count--;
	} else {
		entry = kmem_cache_alloc(swapped.cache, GFP_ATOMIC);
		if (unlikely(!entry))
			/* bad, can't allocate more mem */
			goto out_locked;
	}

	index = page_private(page);
	entry->swp_entry.val = index;
	/*
	 * On numa we need to store the node id to ensure that we prefetch to
	 * the same node it came from.
	 */
	store_swap_entry_node(entry, page);

	if (likely(!radix_tree_insert(&swapped.swap_tree, index, entry))) {
		list_add(&entry->swapped_list, &swapped.list);
		swapped.count++;
	} else
		kmem_cache_free(swapped.cache, entry);

out_locked:
	spin_unlock_irqrestore(&swapped.lock, flags);
out:
	if (!kprefetchd_awake)
		wake_up_process(kprefetchd_task);
	return;
}

/*
 * Removes entries from the swapped_list. The radix tree allows us to quickly
 * look up the entry from the index without having to iterate over the whole
 * list.
 */
static void remove_from_swapped_list(const unsigned long index)
{
	struct swapped_entry *entry;
	unsigned long flags;

	spin_lock_irqsave(&swapped.lock, flags);
	entry = radix_tree_delete(&swapped.swap_tree, index);
	if (likely(entry)) {
		list_del(&entry->swapped_list);
		swapped.count--;
		kmem_cache_free(swapped.cache, entry);
	}
	spin_unlock_irqrestore(&swapped.lock, flags);
}

enum trickle_return {
	TRICKLE_SUCCESS,
	TRICKLE_FAILED,
	TRICKLE_DELAY,
};

struct node_stats {
	/* Free ram after a cycle of prefetching */
	unsigned long	last_free;
	/* Free ram on this cycle of checking prefetch_suitable */
	unsigned long	current_free;
	/* The amount of free ram before we start prefetching */
	unsigned long	highfree[MAX_NR_ZONES];
	/* The amount of free ram where we will stop prefetching */
	unsigned long	lowfree[MAX_NR_ZONES];
	/* highfree or lowfree depending on whether we've hit a watermark */
	unsigned long	*pointfree[MAX_NR_ZONES];
};

/*
 * prefetch_stats stores the free ram data of each node and this is used to
 * determine if a node is suitable for prefetching into.
 */
struct prefetch_stats {
	/* Which nodes are currently suited to prefetching */
	nodemask_t	prefetch_nodes;
	/* Total pages we've prefetched on this wakeup of kprefetchd */
	unsigned long	prefetched_pages;
	struct node_stats node[MAX_NUMNODES];
};

static struct prefetch_stats sp_stat;

/*
 * This tries to read a swp_entry_t into swap cache for swap prefetching.
 * If it returns TRICKLE_DELAY we should delay further prefetching.
 */
static enum trickle_return trickle_swap_cache_async(const swp_entry_t entry,
	const int node)
{
	enum trickle_return ret = TRICKLE_FAILED;
	unsigned long flags;
	struct page *page;

	read_lock_irqsave(&swapper_space.tree_lock, flags);
	/* Entry may already exist */
	page = radix_tree_lookup(&swapper_space.page_tree, entry.val);
	read_unlock_irqrestore(&swapper_space.tree_lock, flags);
	if (page)
		goto out;

	/*
	 * Get a new page to read from swap. We have already checked the
	 * watermarks so __alloc_pages will not call on reclaim.
	 */
	page = alloc_pages_node(node, GFP_HIGHUSER & ~__GFP_WAIT, 0);
	if (unlikely(!page)) {
		ret = TRICKLE_DELAY;
		goto out;
	}

	if (add_to_swap_cache(page, entry)) {
		/* Failed to add to swap cache */
		goto out_release;
	}

	/* Add them to the tail of the inactive list to preserve LRU order */
	lru_cache_add_tail(page);
	if (unlikely(swap_readpage(NULL, page)))
		goto out_release;

	sp_stat.prefetched_pages++;
	sp_stat.node[node].last_free--;

	ret = TRICKLE_SUCCESS;
out_release:
	page_cache_release(page);
out:
	/*
	 * All entries are removed here lazily. This avoids the cost of
	 * remove_from_swapped_list during normal swapin. Thus there are
	 * usually many stale entries.
	 */
	remove_from_swapped_list(entry.val);
	return ret;
}

static void clear_last_prefetch_free(void)
{
	int node;

	/*
	 * Reset the nodes suitable for prefetching to all nodes. We could
	 * update the data to take into account memory hotplug if desired..
	 */
	sp_stat.prefetch_nodes = node_online_map;
	for_each_node_mask(node, sp_stat.prefetch_nodes) {
		struct node_stats *ns = &sp_stat.node[node];

		ns->last_free = 0;
	}
}

static void clear_current_prefetch_free(void)
{
	int node;

	sp_stat.prefetch_nodes = node_online_map;
	for_each_node_mask(node, sp_stat.prefetch_nodes) {
		struct node_stats *ns = &sp_stat.node[node];

		ns->current_free = 0;
	}
}

/*
 * This updates the high and low watermarks of amount of free ram in each
 * node used to start and stop prefetching. We prefetch from pages_high * 4
 * down to pages_high * 3.
 */
static void examine_free_limits(void)
{
	struct zone *z;

	for_each_zone(z) {
		struct node_stats *ns;
		int idx;

		if (!populated_zone(z))
			continue;

		ns = &sp_stat.node[zone_to_nid(z)];
		idx = zone_idx(z);
		ns->lowfree[idx] = z->pages_high * 3;
		ns->highfree[idx] = ns->lowfree[idx] + z->pages_high;

		if (zone_page_state(z, NR_FREE_PAGES) > ns->highfree[idx]) {
			/*
			 * We've gotten above the high watermark of free pages
			 * so we can start prefetching till we get to the low
			 * watermark.
			 */
			ns->pointfree[idx] = &ns->lowfree[idx];
		}
	}
}

/*
 * We want to be absolutely certain it's ok to start prefetching.
 */
static enum trickle_return prefetch_suitable(void)
{
	enum trickle_return ret = TRICKLE_DELAY;
	struct zone *z;
	int node;

	/*
	 * If swap_prefetch is set to a high value we can ignore load
	 * and prefetch whenever we can. Otherwise we test for vm and
	 * cpu activity.
	 */
	if (swap_prefetch < 3) {
		/* Purposefully racy, may return false positive */
		if (test_bit(0, &swapped.busy)) {
			__clear_bit(0, &swapped.busy);
			goto out;
		}

		/*
		 * above_background_load is expensive so we only perform it
		 * every SWAP_CLUSTER_MAX prefetched_pages.
		 * We test to see if we're above_background_load as disk
		 * activity even at low priority can cause interrupt induced
		 * scheduling latencies.
		 */
		if (!(sp_stat.prefetched_pages % SWAP_CLUSTER_MAX) &&
		    above_background_load())
			goto out;
	}
	clear_current_prefetch_free();

	/*
	 * Have some hysteresis between where page reclaiming and prefetching
	 * will occur to prevent ping-ponging between them.
	 */
	for_each_zone(z) {
		struct node_stats *ns;
		unsigned long free;
		int idx;

		if (!populated_zone(z))
			continue;

		node = zone_to_nid(z);
		ns = &sp_stat.node[node];
		idx = zone_idx(z);

		free = zone_page_state(z, NR_FREE_PAGES);
		if (free < *ns->pointfree[idx]) {
			/*
			 * Free pages have dropped below the low watermark so
			 * we won't start prefetching again till we hit the
			 * high watermark of free pages.
			 */
			ns->pointfree[idx] = &ns->highfree[idx];
			node_clear(node, sp_stat.prefetch_nodes);
			continue;
		}
		ns->current_free += free;
	}

	/*
	 * We iterate over each node testing to see if it is suitable for
	 * prefetching and clear the nodemask if it is not.
	 */
	for_each_node_mask(node, sp_stat.prefetch_nodes) {
		struct node_stats *ns = &sp_stat.node[node];

		/*
		 * We check to see that pages are not being allocated
		 * elsewhere at any significant rate implying any
		 * degree of memory pressure (eg during file reads)
		 */
		if (ns->last_free) {
			if (ns->current_free + SWAP_CLUSTER_MAX <
			    ns->last_free) {
				ns->last_free = ns->current_free;
				node_clear(node,
					sp_stat.prefetch_nodes);
				continue;
			}
		} else
			ns->last_free = ns->current_free;

		/* We shouldn't prefetch when we are doing writeback */
		if (node_page_state(node, NR_WRITEBACK))
			node_clear(node, sp_stat.prefetch_nodes);
	}

	/* Nothing suitable, put kprefetchd back to sleep */
	if (nodes_empty(sp_stat.prefetch_nodes))
		return TRICKLE_FAILED;

	/* Survived all that? Hooray we can prefetch! */
	ret = TRICKLE_SUCCESS;
out:
	return ret;
}

/*
 * trickle_swap is the main function that initiates the swap prefetching. It
 * first checks to see if the busy flag is set, and does not prefetch if it
 * is, as the flag implied we are low on memory or swapping in currently.
 * Otherwise it runs until prefetch_suitable fails which occurs when the
 * vm is busy, we prefetch to the watermark, the list is empty or we have
 * iterated over all entries once.
 */
static enum trickle_return trickle_swap(void)
{
	enum trickle_return suitable, ret = TRICKLE_DELAY;
	struct swapped_entry *pos, *n;
	unsigned long flags;

	if (!prefetch_enabled())
		return ret;

	examine_free_limits();
	suitable = prefetch_suitable();
	if (suitable != TRICKLE_SUCCESS)
		return suitable;
	if (list_empty(&swapped.list)) {
		kprefetchd_awake = 0;
		return TRICKLE_FAILED;
	}

	spin_lock_irqsave(&swapped.lock, flags);
	list_for_each_entry_safe_reverse(pos, n, &swapped.list, swapped_list) {
		swp_entry_t swp_entry;
		int node;

		spin_unlock_irqrestore(&swapped.lock, flags);
		cond_resched();
		suitable = prefetch_suitable();
		if (suitable != TRICKLE_SUCCESS) {
			ret = suitable;
			goto out_unlocked;
		}

		spin_lock_irqsave(&swapped.lock, flags);
		if (unlikely(!pos))
			continue;
		node = get_swap_entry_node(pos);
		if (!node_isset(node, sp_stat.prefetch_nodes)) {
			/*
			 * We found an entry that belongs to a node that is
			 * not suitable for prefetching so skip it.
			 */
			continue;
		}
		swp_entry = pos->swp_entry;
		spin_unlock_irqrestore(&swapped.lock, flags);

		if (trickle_swap_cache_async(swp_entry, node) == TRICKLE_DELAY)
			goto out_unlocked;
		spin_lock_irqsave(&swapped.lock, flags);
	}
	spin_unlock_irqrestore(&swapped.lock, flags);

out_unlocked:
	if (sp_stat.prefetched_pages) {
		lru_add_drain();
		sp_stat.prefetched_pages = 0;
	}
	return ret;
}

static int kprefetchd(void *__unused)
{
	struct sched_param param = { .sched_priority = 0 };

	sched_setscheduler(current, SCHED_BATCH, &param);
	set_user_nice(current, 19);
	/* Set ioprio to lowest if supported by i/o scheduler */
	sys_ioprio_set(IOPRIO_WHO_PROCESS, IOPRIO_BE_NR - 1, IOPRIO_CLASS_BE);

	while (!kthread_should_stop()) {
		try_to_freeze();

		if (!kprefetchd_awake) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule();
			kprefetchd_awake = 1;
		}

		if (trickle_swap() == TRICKLE_FAILED)
			schedule_timeout_interruptible(PREFETCH_SLEEP);
		else
			schedule_timeout_interruptible(PREFETCH_DELAY);
		clear_last_prefetch_free();
	}
	return 0;
}

/*
 * Create kmem cache for swapped entries
 */
void __init prepare_swap_prefetch(void)
{
	struct zone *zone;

	swapped.cache = kmem_cache_create("swapped_entry",
		sizeof(struct swapped_entry), 0, SLAB_PANIC, NULL, NULL);

	/*
	 * We set the limit to more entries than the physical ram.
	 * We remove entries lazily so we need some headroom.
	 */
	swapped.maxcount = nr_free_pagecache_pages() * 2;

	for_each_zone(zone) {
		struct node_stats *ns;
		int idx;

		if (!populated_zone(zone))
			continue;

		ns = &sp_stat.node[zone_to_nid(zone)];
		idx = zone_idx(zone);
		ns->pointfree[idx] = &ns->highfree[idx];
	}
}

static int __init kprefetchd_init(void)
{
	kprefetchd_task = kthread_run(kprefetchd, NULL, "kprefetchd");

	return 0;
}

static void __exit kprefetchd_exit(void)
{
	kthread_stop(kprefetchd_task);
}

module_init(kprefetchd_init);
module_exit(kprefetchd_exit);
