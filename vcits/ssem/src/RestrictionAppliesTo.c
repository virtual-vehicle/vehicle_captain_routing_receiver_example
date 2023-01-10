/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/tmp/gen_env/build/asn1/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -D /tmp/gen_env/vcits/ssem -R -no-gen-example -fcompound-names -fno-include-deps -pdu=SSEM`
 */

#include "RestrictionAppliesTo.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
static asn_oer_constraints_t asn_OER_type_RestrictionAppliesTo_constr_1 CC_NOTUSED = {
	{ 0, 0 },
	-1};
asn_per_constraints_t asn_PER_type_RestrictionAppliesTo_constr_1 CC_NOTUSED = {
	{ APC_CONSTRAINED | APC_EXTENSIBLE,  4,  4,  0,  13 }	/* (0..13,...) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static const asn_INTEGER_enum_map_t asn_MAP_RestrictionAppliesTo_value2enum_1[] = {
	{ 0,	4,	"none" },
	{ 1,	15,	"equippedTransit" },
	{ 2,	13,	"equippedTaxis" },
	{ 3,	13,	"equippedOther" },
	{ 4,	17,	"emissionCompliant" },
	{ 5,	15,	"equippedBicycle" },
	{ 6,	15,	"weightCompliant" },
	{ 7,	15,	"heightCompliant" },
	{ 8,	11,	"pedestrians" },
	{ 9,	17,	"slowMovingPersons" },
	{ 10,	15,	"wheelchairUsers" },
	{ 11,	18,	"visualDisabilities" },
	{ 12,	17,	"audioDisabilities" },
	{ 13,	24,	"otherUnknownDisabilities" }
	/* This list is extensible */
};
static const unsigned int asn_MAP_RestrictionAppliesTo_enum2value_1[] = {
	12,	/* audioDisabilities(12) */
	4,	/* emissionCompliant(4) */
	5,	/* equippedBicycle(5) */
	3,	/* equippedOther(3) */
	2,	/* equippedTaxis(2) */
	1,	/* equippedTransit(1) */
	7,	/* heightCompliant(7) */
	0,	/* none(0) */
	13,	/* otherUnknownDisabilities(13) */
	8,	/* pedestrians(8) */
	9,	/* slowMovingPersons(9) */
	11,	/* visualDisabilities(11) */
	6,	/* weightCompliant(6) */
	10	/* wheelchairUsers(10) */
	/* This list is extensible */
};
const asn_INTEGER_specifics_t asn_SPC_RestrictionAppliesTo_specs_1 = {
	asn_MAP_RestrictionAppliesTo_value2enum_1,	/* "tag" => N; sorted by tag */
	asn_MAP_RestrictionAppliesTo_enum2value_1,	/* N => "tag"; sorted by N */
	14,	/* Number of elements in the maps */
	15,	/* Extensions before this member */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_RestrictionAppliesTo_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
asn_TYPE_descriptor_t asn_DEF_RestrictionAppliesTo = {
	"RestrictionAppliesTo",
	"RestrictionAppliesTo",
	&asn_OP_NativeEnumerated,
	asn_DEF_RestrictionAppliesTo_tags_1,
	sizeof(asn_DEF_RestrictionAppliesTo_tags_1)
		/sizeof(asn_DEF_RestrictionAppliesTo_tags_1[0]), /* 1 */
	asn_DEF_RestrictionAppliesTo_tags_1,	/* Same as above */
	sizeof(asn_DEF_RestrictionAppliesTo_tags_1)
		/sizeof(asn_DEF_RestrictionAppliesTo_tags_1[0]), /* 1 */
	{ &asn_OER_type_RestrictionAppliesTo_constr_1, &asn_PER_type_RestrictionAppliesTo_constr_1, NativeEnumerated_constraint },
	0, 0,	/* Defined elsewhere */
	&asn_SPC_RestrictionAppliesTo_specs_1	/* Additional specs */
};

