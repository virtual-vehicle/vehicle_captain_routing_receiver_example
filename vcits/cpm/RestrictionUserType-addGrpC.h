/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "AddGrpC"
 * 	found in "/tmp/gen_env/build/asn1/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -D /tmp/gen_env/vcits/cpm -R -no-gen-example -fcompound-names -fno-include-deps -pdu=CPM`
 */

#ifndef	_RestrictionUserType_addGrpC_H_
#define	_RestrictionUserType_addGrpC_H_


#include <asn_application.h>

/* Including external dependencies */
#include "EmissionType.h"
#include "FuelType.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* RestrictionUserType-addGrpC */
typedef struct RestrictionUserType_addGrpC {
	EmissionType_t	*emission;	/* OPTIONAL */
	FuelType_t	*fuel;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} RestrictionUserType_addGrpC_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_RestrictionUserType_addGrpC;
extern asn_SEQUENCE_specifics_t asn_SPC_RestrictionUserType_addGrpC_specs_1;
extern asn_TYPE_member_t asn_MBR_RestrictionUserType_addGrpC_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _RestrictionUserType_addGrpC_H_ */
#include <asn_internal.h>
