/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IVI"
 * 	found in "/tmp/gen_env/build/asn1/ISO19321IVIv2.asn"
 * 	`asn1c -D /tmp/gen_env/vcits/ivim -R -no-gen-example -fcompound-names -fno-include-deps -pdu=IVIM`
 */

#include "GeographicLocationContainer.h"

#include "Heading.h"
#include "Speed.h"
asn_TYPE_member_t asn_MBR_GeographicLocationContainer_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct GeographicLocationContainer, referencePosition),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ReferencePosition,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"referencePosition"
		},
	{ ATF_POINTER, 3, offsetof(struct GeographicLocationContainer, referencePositionTime),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_TimestampIts,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"referencePositionTime"
		},
	{ ATF_POINTER, 2, offsetof(struct GeographicLocationContainer, referencePositionHeading),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_Heading,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"referencePositionHeading"
		},
	{ ATF_POINTER, 1, offsetof(struct GeographicLocationContainer, referencePositionSpeed),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_Speed,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"referencePositionSpeed"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct GeographicLocationContainer, parts),
		(ASN_TAG_CLASS_CONTEXT | (4 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_GlcParts,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"parts"
		},
};
static const int asn_MAP_GeographicLocationContainer_oms_1[] = { 1, 2, 3 };
static const ber_tlv_tag_t asn_DEF_GeographicLocationContainer_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_GeographicLocationContainer_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* referencePosition */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* referencePositionTime */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* referencePositionHeading */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 }, /* referencePositionSpeed */
    { (ASN_TAG_CLASS_CONTEXT | (4 << 2)), 4, 0, 0 } /* parts */
};
asn_SEQUENCE_specifics_t asn_SPC_GeographicLocationContainer_specs_1 = {
	sizeof(struct GeographicLocationContainer),
	offsetof(struct GeographicLocationContainer, _asn_ctx),
	asn_MAP_GeographicLocationContainer_tag2el_1,
	5,	/* Count of tags in the map */
	asn_MAP_GeographicLocationContainer_oms_1,	/* Optional members */
	3, 0,	/* Root/Additions */
	5,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_GeographicLocationContainer = {
	"GeographicLocationContainer",
	"GeographicLocationContainer",
	&asn_OP_SEQUENCE,
	asn_DEF_GeographicLocationContainer_tags_1,
	sizeof(asn_DEF_GeographicLocationContainer_tags_1)
		/sizeof(asn_DEF_GeographicLocationContainer_tags_1[0]), /* 1 */
	asn_DEF_GeographicLocationContainer_tags_1,	/* Same as above */
	sizeof(asn_DEF_GeographicLocationContainer_tags_1)
		/sizeof(asn_DEF_GeographicLocationContainer_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_GeographicLocationContainer_1,
	5,	/* Elements count */
	&asn_SPC_GeographicLocationContainer_specs_1	/* Additional specs */
};

