/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "VC-Container"
 * 	found in "/tmp/gen_env/../vehicle_captain_its_asn1_specifications/custom/VC-Container-PDU-Descriptions.asn"
 * 	`asn1c -D /tmp/gen_env/vcits/vccontainer -R -no-gen-example -fcompound-names -fno-include-deps -pdu=RawRxMessage -pdu=RawTxMessage`
 */

#include "RawTxMessage.h"

static asn_TYPE_member_t asn_MBR_RawTxMessage_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct RawTxMessage, txInfo),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_TxInfo,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"txInfo"
		},
};
static const ber_tlv_tag_t asn_DEF_RawTxMessage_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_RawTxMessage_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 } /* txInfo */
};
static asn_SEQUENCE_specifics_t asn_SPC_RawTxMessage_specs_1 = {
	sizeof(struct RawTxMessage),
	offsetof(struct RawTxMessage, _asn_ctx),
	asn_MAP_RawTxMessage_tag2el_1,
	1,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_RawTxMessage = {
	"RawTxMessage",
	"RawTxMessage",
	&asn_OP_SEQUENCE,
	asn_DEF_RawTxMessage_tags_1,
	sizeof(asn_DEF_RawTxMessage_tags_1)
		/sizeof(asn_DEF_RawTxMessage_tags_1[0]), /* 1 */
	asn_DEF_RawTxMessage_tags_1,	/* Same as above */
	sizeof(asn_DEF_RawTxMessage_tags_1)
		/sizeof(asn_DEF_RawTxMessage_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_RawTxMessage_1,
	1,	/* Elements count */
	&asn_SPC_RawTxMessage_specs_1	/* Additional specs */
};

