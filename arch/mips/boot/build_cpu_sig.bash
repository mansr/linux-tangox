#!/bin/bash
set -e
#
if [ $# != 3 ]; then 
    cat <<EOF

Syntax: $0 cpufile 000b 8634_ES4_dev

To sign the cpufile with given private key indicated by cert_id/cert_type, 
the certificate id you plan to use, and the chip revision (ES1_dev or ES4_dev) .
also make sure that the scripts below are accessible and in your PATH variable
EOF
    exit -1 
fi
#
if [ -z "$XSDK_ROOT" ]; then 
	if [ -d /utils/em8xxx/signed_items ]; then
		# Use default
		XSDK_ROOT=/utils/em8xxx/signed_items
	else
		echo "*** You need to define the XSDK_ROOT variable ***"
		exit -1
	fi
fi

CERTID=$2
REV=$3
if [ ! "$BOOTAUTH_KEY" = "" -a -f "$BOOTAUTH_KEY" ]; then
	PRIVATE_KEY=${BOOTAUTH_KEY}
else
	PRIVATE_KEY=${XSDK_ROOT}/dummy_private_keys/${REV}_${CERTID}_keyboth.pem
fi

echo "Generating signature with bootauth key = " ${PRIVATE_KEY}
openssl dgst -sha1 -binary -sign ${PRIVATE_KEY} < $1 > $1.sig

