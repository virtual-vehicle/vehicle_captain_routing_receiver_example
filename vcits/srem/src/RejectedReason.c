/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "AddGrpC"
 * 	found in "/tmp/gen_env/build/asn1/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -D /tmp/gen_env/vcits/srem -R -no-gen-example -fcompound-names -fno-include-deps -pdu=SREM`
 */

#include "RejectedReason.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
static asn_oer_constraints_t asn_OER_type_RejectedReason_constr_1 CC_NOTUSED = {
	{ 0, 0 },
	-1};
asn_per_constraints_t asn_PER_type_RejectedReason_constr_1 CC_NOTUSED = {
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  3,  3,  0,  5 }	/* (0..5,...) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static const asn_INTEGER_enum_map_t asn_MAP_RejectedReason_value2enum_1[] = {
	{ 0,	7,	"unknown" },
	{ 1,	20,	"exceptionalCondition" },
	{ 2,	22,	"maxWaitingTimeExceeded" },
	{ 3,	18,	"ptPriorityDisabled" },
	{ 4,	23,	"higherPTPriorityGranted" },
	{ 5,	22,	"vehicleTrackingUnknown" }
	/* This list is extensible */
};
static const unsigned int asn_MAP_RejectedReason_enum2value_1[] = {
	1,	/* exceptionalCondition(1) */
	4,	/* higherPTPriorityGranted(4) */
	2,	/* maxWaitingTimeExceeded(2) */
	3,	/* ptPriorityDisabled(3) */
	0,	/* unknown(0) */
	5	/* vehicleTrackingUnknown(5) */
	/* This list is extensible */
};
const asn_INTEGER_specifics_t asn_SPC_RejectedReason_specs_1 = {
	asn_MAP_RejectedReason_value2enum_1,	/* "tag" => N; sorted by tag */
	asn_MAP_RejectedReason_enum2value_1,	/* N => "tag"; sorted by N */
	6,	/* Number of elements in the maps */
	7,	/* Extensions before this member */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_RejectedReason_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
asn_TYPE_descriptor_t asn_DEF_RejectedReason = {
	"RejectedReason",
	"RejectedReason",
	&asn_OP_NativeEnumerated,
	asn_DEF_RejectedReason_tags_1,
	sizeof(asn_DEF_RejectedReason_tags_1)
		/sizeof(asn_DEF_RejectedReason_tags_1[0]), /* 1 */
	asn_DEF_RejectedReason_tags_1,	/* Same as above */
	sizeof(asn_DEF_RejectedReason_tags_1)
		/sizeof(asn_DEF_RejectedReason_tags_1[0]), /* 1 */
	{ &asn_OER_type_RejectedReason_constr_1, &asn_PER_type_RejectedReason_constr_1, NativeEnumerated_constraint },
	0, 0,	/* Defined elsewhere */
	&asn_SPC_RejectedReason_specs_1	/* Additional specs */
};

