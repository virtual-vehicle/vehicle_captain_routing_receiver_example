/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/tmp/gen_env/../vehicle_captain_its_asn1_specifications/etsi/cdd_ts102894_2/ITS-Container.asn"
 * 	`asn1c -D /tmp/gen_env/vcits/ssem -R -no-gen-example -fcompound-names -fno-include-deps -pdu=SSEM`
 */

#include "ItineraryPath.h"

#include "ReferencePosition.h"
static asn_oer_constraints_t asn_OER_type_ItineraryPath_constr_1 CC_NOTUSED = {
	{ 0, 0 },
	-1	/* (SIZE(1..40)) */};
static asn_per_constraints_t asn_PER_type_ItineraryPath_constr_1 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_CONSTRAINED,	 6,  6,  1,  40 }	/* (SIZE(1..40)) */,
	0, 0	/* No PER value map */
};
static asn_TYPE_member_t asn_MBR_ItineraryPath_1[] = {
	{ ATF_POINTER, 0, 0,
		(ASN_TAG_CLASS_UNIVERSAL | (16 << 2)),
		0,
		&asn_DEF_ReferencePosition,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		""
		},
};
static const ber_tlv_tag_t asn_DEF_ItineraryPath_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static asn_SET_OF_specifics_t asn_SPC_ItineraryPath_specs_1 = {
	sizeof(struct ItineraryPath),
	offsetof(struct ItineraryPath, _asn_ctx),
	0,	/* XER encoding is XMLDelimitedItemList */
};
asn_TYPE_descriptor_t asn_DEF_ItineraryPath = {
	"ItineraryPath",
	"ItineraryPath",
	&asn_OP_SEQUENCE_OF,
	asn_DEF_ItineraryPath_tags_1,
	sizeof(asn_DEF_ItineraryPath_tags_1)
		/sizeof(asn_DEF_ItineraryPath_tags_1[0]), /* 1 */
	asn_DEF_ItineraryPath_tags_1,	/* Same as above */
	sizeof(asn_DEF_ItineraryPath_tags_1)
		/sizeof(asn_DEF_ItineraryPath_tags_1[0]), /* 1 */
	{ &asn_OER_type_ItineraryPath_constr_1, &asn_PER_type_ItineraryPath_constr_1, SEQUENCE_OF_constraint },
	asn_MBR_ItineraryPath_1,
	1,	/* Single element */
	&asn_SPC_ItineraryPath_specs_1	/* Additional specs */
};

