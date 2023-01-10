/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CITSapplMgmtIDs"
 * 	found in "/tmp/gen_env/build/asn1/TS17419_2014_CITSapplMgmtIDs.asn"
 * 	`asn1c -D /tmp/gen_env/vcits/ivim -R -no-gen-example -fcompound-names -fno-include-deps -pdu=IVIM`
 */

#include "VarLengthNumber2.h"

static int
memb_shortNo_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 0 && value <= 127)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static int
memb_longNo_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= 0 && value <= 32767)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static asn_oer_constraints_t asn_OER_memb_shortNo_constr_2 CC_NOTUSED = {
	{ 1, 1 }	/* (0..127) */,
	-1};
static asn_per_constraints_t asn_PER_memb_shortNo_constr_2 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 7,  7,  0,  127 }	/* (0..127) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_memb_longNo_constr_3 CC_NOTUSED = {
	{ 2, 1 }	/* (0..32767) */,
	-1};
static asn_per_constraints_t asn_PER_memb_longNo_constr_3 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 15,  15,  0,  32767 }	/* (0..32767) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_oer_constraints_t asn_OER_type_VarLengthNumber2_constr_1 CC_NOTUSED = {
	{ 0, 0 },
	-1};
static asn_per_constraints_t asn_PER_type_VarLengthNumber2_constr_1 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 1,  1,  0,  1 }	/* (0..1) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
static asn_TYPE_member_t asn_MBR_VarLengthNumber2_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct VarLengthNumber2, choice.shortNo),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		0,
		{ &asn_OER_memb_shortNo_constr_2, &asn_PER_memb_shortNo_constr_2,  memb_shortNo_constraint_1 },
		0, 0, /* No default value */
		"shortNo"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct VarLengthNumber2, choice.longNo),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		0,
		{ &asn_OER_memb_longNo_constr_3, &asn_PER_memb_longNo_constr_3,  memb_longNo_constraint_1 },
		0, 0, /* No default value */
		"longNo"
		},
};
static const asn_TYPE_tag2member_t asn_MAP_VarLengthNumber2_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* shortNo */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* longNo */
};
static asn_CHOICE_specifics_t asn_SPC_VarLengthNumber2_specs_1 = {
	sizeof(struct VarLengthNumber2),
	offsetof(struct VarLengthNumber2, _asn_ctx),
	offsetof(struct VarLengthNumber2, present),
	sizeof(((struct VarLengthNumber2 *)0)->present),
	asn_MAP_VarLengthNumber2_tag2el_1,
	2,	/* Count of tags in the map */
	0, 0,
	-1	/* Extensions start */
};
asn_TYPE_descriptor_t asn_DEF_VarLengthNumber2 = {
	"VarLengthNumber2",
	"VarLengthNumber2",
	&asn_OP_CHOICE,
	0,	/* No effective tags (pointer) */
	0,	/* No effective tags (count) */
	0,	/* No tags (pointer) */
	0,	/* No tags (count) */
	{ &asn_OER_type_VarLengthNumber2_constr_1, &asn_PER_type_VarLengthNumber2_constr_1, CHOICE_constraint },
	asn_MBR_VarLengthNumber2_1,
	2,	/* Elements count */
	&asn_SPC_VarLengthNumber2_specs_1	/* Additional specs */
};

