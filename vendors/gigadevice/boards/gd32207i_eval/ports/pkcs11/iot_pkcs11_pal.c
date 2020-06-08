/*
 * Amazon FreeRTOS PKCS #11 PAL for Numaker_PFM_M487 V1.0.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
 */


/**
 * @file aws_pkcs11_pal.c
 * @brief Amazon FreeRTOS device specific helper functions for
 * PKCS#11 implementation based on mbedTLS.  This
 * file deviates from the FreeRTOS style standard for some function names and
 * data types in order to maintain compliance with the PKCS#11 standard.
 */

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "FreeRTOSIPConfig.h"
#include "task.h"
#include "iot_pkcs11.h"
#include "iot_pkcs11_config.h"

/* C runtime includes. */
#include <stdio.h>
#include <string.h>

/* flash driver includes. */
#include "gd32f20x_libopt.h"

#define PKCS11_CLIENT_CERTIFICATE_MARK           "PKCS11_Certificate"
#define PKCS11_PRIVITE_KEY_MARK                  "PKCS11_PRIVITE_Key"
#define PKCS11_CODE_SIGN_PUBLIC_KEY_MARK         "PKCS11_CodeSignKey"

#define pkcs11OBJECT_MAX_SIZE                    ( 0x800 )
#define pkcs11OBJECT_FLASH_MARK                  ( 0x5A5A5A5AUL )

enum eObjectHandles
{
    eInvalidHandle = 0, /* From PKCS #11 spec: 0 is never a valid object handle.*/
    eAwsDevicePrivateKey = 1,
    eAwsDevicePublicKey,
    eAwsDeviceCertificate,
    eAwsCodeSigningKey
};

/* @brief Structure for certificates/key storage */
typedef struct
{
    CK_ULONG datamark;
    CK_ULONG datasize;
    CK_CHAR p11_data[ pkcs11OBJECT_MAX_SIZE ];
} pkcs11_data_t;

typedef struct
{
    uint32_t DeviceCertificate;
    uint32_t DeviceKey;
    uint32_t CodeSignKey;
    uint32_t Reserve;
} pkcs11_config_t;

/* @brief Certificates/key storage in flash */
#define GD_STORE_BASE             ( 0x08100000UL )      /* local storage start address in FLASH */
#define PAGE_SIZE                 ( 0x00001000UL )      /* size of flash page(4K) */

/* Set the first 4 pages of bank1 as the certification storage, PAGE_SIZE=4KB */
static pkcs11_config_t p11keyconfig ={ GD_STORE_BASE, 
                                      GD_STORE_BASE + PAGE_SIZE,
                                      GD_STORE_BASE + PAGE_SIZE*2,
                                      GD_STORE_BASE + PAGE_SIZE*3};

static pkcs11_data_t p11datasave;

/*-----------------------------------------------------------*/



/*-----------------------------------------------------------*/

/**
 * @brief write data in to flash.
 *
 * Port-specific file write for crytographic information.
 *
 * @param[in] start_addr    The address of the data to be written to.
 * @param[in] srcdata       Data to be written to flash
 * @param[in] size          Size of the data(in bytes).
 *
 * @return the size of the written data.
 */
static uint32_t flash_update( uint32_t start_addr, 
    uint8_t * srcdata,
    uint32_t size )
{
    uint32_t temp_addr;             /* flash address       */
    uint32_t *pdata;                /* flash data          */
    uint32_t end_addr = ( start_addr + sizeof( pkcs11_data_t ) );

    /* unlock flash memory */
    fmc_unlock();

    /* erase flash sector and clear completion flag */
    fmc_page_erase( start_addr );
    fmc_flag_clear( FMC_INT_FLAG_BANK1_END );

    /* write client certificate or key */
    memcpy( (void *)p11datasave.p11_data, (void *)start_addr, pkcs11OBJECT_MAX_SIZE );
    memcpy( p11datasave.p11_data, srcdata, size );
    p11datasave.datamark = pkcs11OBJECT_FLASH_MARK;
    p11datasave.datasize = size;
    pdata = (uint32_t *) &p11datasave;
    /* Fill flash range from u32StartAddr to u32EndAddr with data word u32Pattern. */
    for ( temp_addr = start_addr; temp_addr < end_addr; temp_addr += 4 )
    {
        /* Program flash */
        fmc_word_program( temp_addr, *pdata );
        pdata++;
    }

    /* lock flash memory */
    fmc_lock();

    return size;
}

/*-----------------------------------------------------------*/

/**
 * @brief Get attribute from the provided label.
 *
 * Port-specific file write for crytographic information.
 *
 * @param[in] pvalue    The name of the label.
 * @param[out] pmark    The mark of the data to be written in to flash.
 * @param[out] phandle  The object handle.
 *
 */
static void get_label_attribute( uint8_t* pvalue,
    char** pmark,
    CK_OBJECT_HANDLE_PTR phandle )
{
    if( pvalue != NULL )
    {
        if( 0 == memcmp( pvalue,
                         &pkcs11configLABEL_DEVICE_CERTIFICATE_FOR_TLS,
                         sizeof( pkcs11configLABEL_DEVICE_CERTIFICATE_FOR_TLS ) ) )
        {
            *pmark = PKCS11_CLIENT_CERTIFICATE_MARK;
            *phandle = eAwsDeviceCertificate;
        }
        else if( 0 == memcmp( pvalue,
                              &pkcs11configLABEL_DEVICE_PRIVATE_KEY_FOR_TLS,
                              sizeof( pkcs11configLABEL_DEVICE_PRIVATE_KEY_FOR_TLS ) ) )
        {
            *pmark = PKCS11_PRIVITE_KEY_MARK;
            *phandle = eAwsDevicePrivateKey;
        }
        else if( 0 == memcmp( pvalue,
                              &pkcs11configLABEL_DEVICE_PUBLIC_KEY_FOR_TLS,
                              sizeof( pkcs11configLABEL_DEVICE_PUBLIC_KEY_FOR_TLS ) ) )
        {
            *pmark = PKCS11_PRIVITE_KEY_MARK;
            *phandle = eAwsDevicePublicKey;
        }
        else if( 0 == memcmp( pvalue,
                              &pkcs11configLABEL_CODE_VERIFICATION_KEY,
                              sizeof( pkcs11configLABEL_CODE_VERIFICATION_KEY ) ) )
        {
            *pmark = PKCS11_CODE_SIGN_PUBLIC_KEY_MARK;
            *phandle = eAwsCodeSigningKey;
        }
        else
        {
            *pmark = NULL;
            *phandle = eInvalidHandle;
        }
    }
}

/*-----------------------------------------------------------*/

/**
 * @brief Write client certificate or keys to local storage.
 *
 * Port-specific file write for crytographic information.
 *
 * @param[in] file_mark     The mark of the file to be written to.
 * @param[in] src_data      Data to be written to flash
 * @param[in] data_size     Size of data(in bytes).
 *
 * @return pdTRUE if data was saved successfully to file,
 * pdFALSE otherwise.
 */
static ErrStatus save_file( char *file_mark,
    uint8_t * src_data,
    uint32_t data_size )
{
    ErrStatus result = ERROR;
    uint32_t flash_addr = 0;
    uint32_t byteswritten = 0;

    /* enough room to store the certificate */
    if( data_size > pkcs11OBJECT_MAX_SIZE )
    {
        return result;
    }

    /* write client certificate or keys */
    if( strcmp( file_mark, PKCS11_CLIENT_CERTIFICATE_MARK ) == 0 )
    {
        flash_addr = p11keyconfig.DeviceCertificate;
    }
    else if( strcmp( file_mark, PKCS11_PRIVITE_KEY_MARK ) == 0 )
    {
        flash_addr = p11keyconfig.DeviceKey;
    }
    else if( strcmp( file_mark, PKCS11_CODE_SIGN_PUBLIC_KEY_MARK ) == 0 )
    {
        flash_addr = p11keyconfig.CodeSignKey;
    }
    else
    {
        flash_addr = NULL;
    }

    if( flash_addr != NULL )
    {
        byteswritten = flash_update( flash_addr, src_data, data_size );
        if( byteswritten == data_size )
        {
            result = SUCCESS;
        }
    }

    return result;
}

/*-----------------------------------------------------------*/

/**
 * @brief Reads a pkcs#11 file from local storage.
 *
 * Port-specific file access for crytographic information.
 *
 * @param[in] file_mark     The mark of the file to be read.
 * @param[out] ppdata       Pointer to buffer for file data.
 * @param[out] pdatasize    Size of data located in file(in bytes).
 *
 * @return pdTRUE if data was retrieved successfully from files,
 * pdFALSE otherwise.
 */
static ErrStatus read_file( char *pkcs_mark,
    uint8_t **ppdata,
    uint32_t *pdatasize )
{
    ErrStatus result = ERROR;
    const pkcs11_data_t * p11data = 0;
    uint32_t p11datasize = 0;
    uint8_t * ptemp = 0;

    /* read client certificate */
    if( strcmp( pkcs_mark, PKCS11_CLIENT_CERTIFICATE_MARK ) == 0 )
    {
        p11data = (pkcs11_data_t *)p11keyconfig.DeviceCertificate;
    }
    else if( strcmp( pkcs_mark, PKCS11_PRIVITE_KEY_MARK ) == 0 )
    {
        p11data = (pkcs11_data_t *)p11keyconfig.DeviceKey;
    }
    else if( strcmp( pkcs_mark, PKCS11_CODE_SIGN_PUBLIC_KEY_MARK ) == 0 )
    {
        p11data = (pkcs11_data_t *)p11keyconfig.CodeSignKey;
    }

    if( ( p11data !=0 ) && ( p11data->datamark == pkcs11OBJECT_FLASH_MARK ) )
    {
        ptemp = ( uint8_t * ) p11data->p11_data;
        p11datasize = p11data->datasize;
        result = SUCCESS;
    }

    *pdatasize = p11datasize;
    *ppdata = ptemp;

    return result;
}

/**
* @brief Writes a file to local storage.
*
* Port-specific file write for crytographic information.
*
* @param[in] pxLabel       Label of the object to be saved.
* @param[in] pucData       Data buffer to be written to file
* @param[in] ulDataSize    Size (in bytes) of data to be saved.
*
* @return The file handle of the object that was stored.
*/
CK_OBJECT_HANDLE PKCS11_PAL_SaveObject( CK_ATTRIBUTE_PTR pxLabel,
    uint8_t * pucData,
    uint32_t ulDataSize )
{
    char *pMark = NULL;
    CK_OBJECT_HANDLE xHandle = eInvalidHandle;

    /* get attribute from the provided label */
    get_label_attribute( pxLabel->pValue, &pMark, &xHandle );

    if( pMark != NULL )
    {
        if( SUCCESS != save_file( pMark, pucData, ulDataSize ) )
        {
            xHandle = eInvalidHandle;
        }
    }

    return xHandle;
}

/*-----------------------------------------------------------*/

/**
* @brief Translates a PKCS #11 label into an object handle.
*
* Port-specific object handle retrieval.
*
*
* @param[in] pLabel         Pointer to the label of the object
*                           who's handle should be found.
* @param[in] usLength       The length of the label, in bytes.
*
* @return The object handle if operation was successful.
* Returns eInvalidHandle if unsuccessful.
*/
CK_OBJECT_HANDLE PKCS11_PAL_FindObject( uint8_t * pLabel,
    uint8_t usLength )
{
    uint8_t *pucData = NULL;
    uint32_t dataSize = 0;
    ( void ) usLength;

    CK_OBJECT_HANDLE xHandle = eInvalidHandle;
    char * pcFileName = NULL;

    /* get attribute from the provided label */
    get_label_attribute( pLabel, &pcFileName, &xHandle );

    if( pcFileName != NULL )
    {
        /* check object does exist */
        if( SUCCESS != read_file( pcFileName, &pucData, &dataSize) )
        {
            xHandle = eInvalidHandle;
        }
    }

    return xHandle;
}

/*-----------------------------------------------------------*/

/**
* @brief Gets the value of an object in storage, by handle.
*
* Port-specific file access for cryptographic information.
*
* This call dynamically allocates the buffer which object value
* data is copied into.  PKCS11_PAL_GetObjectValueCleanup()
* should be called after each use to free the dynamically allocated
* buffer.
*
* @sa PKCS11_PAL_GetObjectValueCleanup
*
* @param[in] pcFileName    The name of the file to be read.
* @param[out] ppucData     Pointer to buffer for file data.
* @param[out] pulDataSize  Size (in bytes) of data located in file.
* @param[out] pIsPrivate   Boolean indicating if value is private (CK_TRUE)
*                          or exportable (CK_FALSE)
*
* @return CKR_OK if operation was successful.  CKR_KEY_HANDLE_INVALID if
* no such object handle was found, CKR_DEVICE_MEMORY if memory for
* buffer could not be allocated, CKR_FUNCTION_FAILED for device driver
* error.
*/
CK_RV PKCS11_PAL_GetObjectValue( CK_OBJECT_HANDLE xHandle,
    uint8_t ** ppucData,
    uint32_t * pulDataSize,
    CK_BBOOL * pIsPrivate )
{

    CK_RV result = CKR_OK;
    char *pc11mark = NULL;

    if( xHandle == eAwsDeviceCertificate )
    {
        pc11mark = PKCS11_CLIENT_CERTIFICATE_MARK;
        *pIsPrivate = CK_FALSE;
    }
    else if( xHandle == eAwsDevicePrivateKey )
    {
        pc11mark = PKCS11_PRIVITE_KEY_MARK;
        *pIsPrivate = CK_TRUE;
    }
    else if( xHandle == eAwsDevicePublicKey )
    {
        pc11mark = PKCS11_PRIVITE_KEY_MARK;
        *pIsPrivate = CK_FALSE;
    }
    else if( xHandle == eAwsCodeSigningKey )
    {
        pc11mark = PKCS11_CODE_SIGN_PUBLIC_KEY_MARK;
        *pIsPrivate = CK_FALSE;
    }
    else
    {
        result = CKR_KEY_HANDLE_INVALID;
    }

    if( pc11mark != NULL )
    {
        /* check object does exist */
        if( SUCCESS != read_file( pc11mark, ppucData, pulDataSize) )
        {
            xHandle = eInvalidHandle;
        }
    }
    return result;
}

/*-----------------------------------------------------------*/

/**
* @brief Cleanup after PKCS11_GetObjectValue().
*
* @param[in] pucData       The buffer to free.
*                          (*ppucData from PKCS11_PAL_GetObjectValue())
* @param[in] ulDataSize    The length of the buffer to free.
*                          (*pulDataSize from PKCS11_PAL_GetObjectValue())
*/
void PKCS11_PAL_GetObjectValueCleanup( uint8_t * pucData,
    uint32_t ulDataSize )
{
    /* Unused parameters. */
    ( void ) pucData;
    ( void ) ulDataSize;

    /* Since no buffer was allocated on heap, there is no cleanup
     * to be done. */
}
