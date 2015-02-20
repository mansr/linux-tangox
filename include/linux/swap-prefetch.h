#ifndef SWAP_PREFETCH_H_INCLUDED
#define SWAP_PREFETCH_H_INCLUDED

#ifdef CONFIG_SWAP_PREFETCH
/* mm/swap_prefetch.c */
extern int swap_prefetch;
extern int swap_prefetch_delay;
extern int swap_prefetch_sleep;

struct swapped_entry {
	swp_entry_t		swp_entry;	/* The actual swap entry */
	struct list_head	swapped_list;	/* Linked list of entries */
#if MAX_NUMNODES > 1
	int			node;		/* Node id */
#endif
} __attribute__((packed));

static inline void store_swap_entry_node(struct swapped_entry *entry,
	struct page *page)
{
#if MAX_NUMNODES > 1
	entry->node = page_to_nid(page);
#endif
}

static inline int get_swap_entry_node(struct swapped_entry *entry)
{
#if MAX_NUMNODES > 1
	return entry->node;
#else
	return 0;
#endif
}

extern void add_to_swapped_list(struct page *page);
extern void delay_swap_prefetch(void);
extern void prepare_swap_prefetch(void);

#else	/* CONFIG_SWAP_PREFETCH */
static inline void add_to_swapped_list(struct page *__unused)
{
}

static inline void prepare_swap_prefetch(void)
{
}

static inline void delay_swap_prefetch(void)
{
}
#endif	/* CONFIG_SWAP_PREFETCH */

#endif		/* SWAP_PREFETCH_H_INCLUDED */
