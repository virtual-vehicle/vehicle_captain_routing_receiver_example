/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "EfcDsrcApplication"
 * 	found in "/tmp/gen_env/../vehicle_captain_its_asn1_specifications/etsi/is_ts103301/iso-patched/ISO14906(2018)EfcDsrcApplicationv6-patched.asn"
 * 	`asn1c -D /tmp/gen_env/vcits/ivim -R -no-gen-example -fcompound-names -fno-include-deps -pdu=IVIM`
 */

#ifndef	_Provider_H_
#define	_Provider_H_


#include <asn_application.h>

/* Including external dependencies */
#include "CountryCode.h"
#include "IssuerIdentifier.h"
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Provider */
typedef struct Provider {
	CountryCode_t	 countryCode;
	IssuerIdentifier_t	 providerIdentifier;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} Provider_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_Provider;
extern asn_SEQUENCE_specifics_t asn_SPC_Provider_specs_1;
extern asn_TYPE_member_t asn_MBR_Provider_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _Provider_H_ */
#include <asn_internal.h>
