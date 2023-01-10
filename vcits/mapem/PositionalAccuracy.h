/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in "/tmp/gen_env/build/asn1/ISO-TS-19091-addgrp-C-2018-patched.asn"
 * 	`asn1c -D /tmp/gen_env/vcits/mapem -R -no-gen-example -fcompound-names -fno-include-deps -pdu=MAPEM`
 */

#ifndef	_PositionalAccuracy_H_
#define	_PositionalAccuracy_H_


#include <asn_application.h>

/* Including external dependencies */
#include "SemiMajorAxisAccuracy.h"
#include "SemiMinorAxisAccuracy.h"
#include "SemiMajorAxisOrientation.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* PositionalAccuracy */
typedef struct PositionalAccuracy {
	SemiMajorAxisAccuracy_t	 semiMajor;
	SemiMinorAxisAccuracy_t	 semiMinor;
	SemiMajorAxisOrientation_t	 orientation;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} PositionalAccuracy_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_PositionalAccuracy;
extern asn_SEQUENCE_specifics_t asn_SPC_PositionalAccuracy_specs_1;
extern asn_TYPE_member_t asn_MBR_PositionalAccuracy_1[3];

#ifdef __cplusplus
}
#endif

#endif	/* _PositionalAccuracy_H_ */
#include <asn_internal.h>
