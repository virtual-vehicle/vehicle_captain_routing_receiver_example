/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/tmp/gen_env/build/asn1/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -D /tmp/gen_env/vcits/cpm -R -no-gen-example -fcompound-names -fno-include-deps -pdu=CPM`
 */

#include "SpeedConfidenceDSRC.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
static asn_oer_constraints_t asn_OER_type_SpeedConfidenceDSRC_constr_1 CC_NOTUSED = {
	{ 0, 0 },
	-1};
asn_per_constraints_t asn_PER_type_SpeedConfidenceDSRC_constr_1 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 3,  3,  0,  7 }	/* (0..7) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static const asn_INTEGER_enum_map_t asn_MAP_SpeedConfidenceDSRC_value2enum_1[] = {
	{ 0,	11,	"unavailable" },
	{ 1,	9,	"prec100ms" },
	{ 2,	8,	"prec10ms" },
	{ 3,	7,	"prec5ms" },
	{ 4,	7,	"prec1ms" },
	{ 5,	9,	"prec0-1ms" },
	{ 6,	10,	"prec0-05ms" },
	{ 7,	10,	"prec0-01ms" }
};
static const unsigned int asn_MAP_SpeedConfidenceDSRC_enum2value_1[] = {
	7,	/* prec0-01ms(7) */
	6,	/* prec0-05ms(6) */
	5,	/* prec0-1ms(5) */
	1,	/* prec100ms(1) */
	2,	/* prec10ms(2) */
	4,	/* prec1ms(4) */
	3,	/* prec5ms(3) */
	0	/* unavailable(0) */
};
const asn_INTEGER_specifics_t asn_SPC_SpeedConfidenceDSRC_specs_1 = {
	asn_MAP_SpeedConfidenceDSRC_value2enum_1,	/* "tag" => N; sorted by tag */
	asn_MAP_SpeedConfidenceDSRC_enum2value_1,	/* N => "tag"; sorted by N */
	8,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_SpeedConfidenceDSRC_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
asn_TYPE_descriptor_t asn_DEF_SpeedConfidenceDSRC = {
	"SpeedConfidenceDSRC",
	"SpeedConfidenceDSRC",
	&asn_OP_NativeEnumerated,
	asn_DEF_SpeedConfidenceDSRC_tags_1,
	sizeof(asn_DEF_SpeedConfidenceDSRC_tags_1)
		/sizeof(asn_DEF_SpeedConfidenceDSRC_tags_1[0]), /* 1 */
	asn_DEF_SpeedConfidenceDSRC_tags_1,	/* Same as above */
	sizeof(asn_DEF_SpeedConfidenceDSRC_tags_1)
		/sizeof(asn_DEF_SpeedConfidenceDSRC_tags_1[0]), /* 1 */
	{ &asn_OER_type_SpeedConfidenceDSRC_constr_1, &asn_PER_type_SpeedConfidenceDSRC_constr_1, NativeEnumerated_constraint },
	0, 0,	/* Defined elsewhere */
	&asn_SPC_SpeedConfidenceDSRC_specs_1	/* Additional specs */
};

