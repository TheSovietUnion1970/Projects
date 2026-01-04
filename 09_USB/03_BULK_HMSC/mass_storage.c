#include "usb1.h"
#include "mass_storage.h"
#include "main.h"
#include <linux/math64.h>
#include <linux/byteorder/generic.h> // for get_unaligned_leXX/beXX

cbw_EAA cbw_EAA_Instance;

/* const CBW */
u8 cbw_initial[31] = {
    0x55, 0x53, 0x42, 0x43,  // dCBWSignature: 0x43425355 (LE "USBC")
    0x01, 0x00, 0x00, 0x00,  // dCBWTag: 0x00000001
    0x24, 0x00, 0x00, 0x00,  // dCBWDataTransferLength: 0x00000024 (36 bytes, Data-In)
    0x80,                    // bmCBWFlags: 0x80 (IN direction)
    0x00,                    // bCBWLUN: 0x00
    0x06,                    // bCBWCBLength: 0x06 (6-byte SCSI cmd)
    0x12, 0x00, 0x00, 0x00, 0x24, 0x00,  // CBWCB: INQUIRY (0x12) + params (LUN=0x00, page=0x00, reserved=0x00, alloc len=0x0024)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Padding (10 zeros to reach 31 bytes)
    0x00, 0x00  // Final padding zeros
};

u8 cbw_capacity[31] = {
    0x55, 0x53, 0x42, 0x43,  // dCBWSignature: 0x43425355 (LE "USBC")
    0x02, 0x00, 0x00, 0x00,  // dCBWTag: 0x00000002 (increment from previous)
    0x08, 0x00, 0x00, 0x00,  // dCBWDataTransferLength: 0x00000008 (8 bytes Data-In)
    0x80,                    // bmCBWFlags: 0x80 (Data-In direction)
    0x00,                    // bCBWLUN: 0x00 (LUN 0)
    0x0A,                    // bCBWCBLength: 0x0A (10-byte SCSI command)
    0x25, 0x00, 0x00, 0x00,  // CBWCB: Opcode 0x25 (READ CAPACITY (10))
    0x00, 0x00, 0x00, 0x00,  // CBWCB: Logical Block Address (0x00000000 for full capacity)
    0x00, 0x00,              // CBWCB: Reserved (0x00) and PMI (0x00, no partial media info)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Padding zeros to reach 31 bytes
};

u8 cbw_mode_sense_EAA[31] = {
    0x55, 0x53, 0x42, 0x43, // dCBWSignature: 0x43425355 ("USBC" in little-endian)
    0x03, 0x00, 0x00, 0x00, // dCBWTag: 0x00000004 (arbitrary, incremented as needed)
    0x18, 0x00, 0x00, 0x00, // dCBWDataTransferLength: 0x00000018 (24 bytes Data-In)
    0x80,                   // bmCBWFlags: 0x80 (Data-In direction)
    0x00,                   // bCBWLUN: 0x00 (LUN 0)
    0x06,                   // bCBWCBLength: 0x06 (6-byte SCSI command)
    0x1A, 0x08, 0x3F, 0x00, // CBWCB: Opcode 0x1A, DBD=1 (0x08), Page Code 0x1D, Subpage 0x00
    0x18, 0x00,             // CBWCB: Allocation Length 0x18 (24 bytes), Control 0x00
    0x00, 0x00, 0x00, 0x00, // Padding zeros (ignored since length=6)
    0x00, 0x00, 0x00, 0x00, // Padding zeros
    0x00, 0x00              // Padding zeros (total CBWCB=16 bytes)
};

u8 cbw_test_ready[31] = {
    0x55, 0x53, 0x42, 0x43,  // Signature
    0x04, 0x00, 0x00, 0x00,  // Tag
    0x00, 0x00, 0x00, 0x00,  // Length=0 (no data)
    0x00,                    // Flags=0x00 (no direction)
    0x00,                    // LUN=0
    0x06,                    // CB length=6
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // CBWCB: All zeros
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  // Padding
};

u32 swap_endian32(u32 val) {
    return ((val >> 24) & 0x000000FF) |
           ((val >> 8)  & 0x0000FF00) |
           ((val << 8)  & 0x00FF0000) |
           ((val << 24) & 0xFF000000);
}

void swap_endian32Ptr(u32 *val) {
    u32 tmp = 0;

    tmp = ((*val >> 24) & 0x000000FF) |
    ((*val >> 8)  & 0x0000FF00) |
    ((*val << 8)  & 0x00FF0000) |
    ((*val << 24) & 0xFF000000);

    *val = tmp;

    //printk("tmp = 0x%x\n", *val);
}

void swap_endian16Ptr(u16 *val) {
    u16 tmp = 0;

    tmp = ((*val >> 8) & 0x00FF) |
    ((*val << 8) & 0xFF00);

    *val = tmp;

    //printk("tmp = 0x%x\n", *val);
}


/* Sector CBW */
//   - CBWCB[0] = 0x28 (READ(10) opcode)
//   - CBWCB[1] = 0x00 (flags: RDPROTECT=0, DPO=0, FUA=0)
//   - CBWCB[2-5] = LBA (big-endian, changeable, e.g., 0x00 0x00 0x00 0x00)
//   - CBWCB[6] = 0x00 (reserved)
//   - CBWCB[7] = 0x00 (group number, changeable)
//   - CBWCB[8-9] = Transfer Length (big-endian, changeable, e.g., 0x00 0x01 for 1 block)
//   - CBWCB[10] = 0x00 (control)
//   - CBWCB[11-15] = 0x00 (padding)
u8 cbw_read_sector[31] = {
    0x55, 0x53, 0x42, 0x43,  // Signature "USBC"
    0x05, 0x00, 0x00, 0x00,  // Tag -> automatically updated
    0x00, 0x02, 0x00, 0x00,  // Data Length: 512 bytes (0x00000200)
    0x80,                    // Data-In
    0x00,                    // LUN 0
    0x0A,                    // CB Length: 10 bytes
    0x28, 0x00,              // Opcode 0x28 (READ(10)), Flags=0x00 (no DPO/FUA)
    0x00, 0x00, 0x00, 0x02,  // LBA: 0x00000000 -> changeable
    0x00,                    // Group Number 0 -> changeable
    0x00,                    // reserved
    0x01, 0x0,               // Transfer Length: 1 block -> changeable (little endian)
    0x00,                    // Control 0x00
    0x00, 0x00, 0x00, 0x00,  // Padding
    0x00                     // Padding
};

u8 cbw_write_sector[31] = {
    0x55, 0x53, 0x42, 0x43, // Signature "USBC"
    0x05, 0x00, 0x00, 0x00, // Tag -> automatically updated
    0x00, 0x02, 0x00, 0x00, // Data Length: 512 bytes (0x00000200, little-endian)
    0x00, // bmCBWFlags: Data-Out (0x00 for host-to-device write)
    0x00, // bCBWLUN: 0
    0x0A, // bCBWCBLength: 10 bytes (for SCSI-10 CDB)
          // SCSI WRITE(10) CDB starts here (big-endian multi-byte fields)
    0x2A, 0x00, // Opcode 0x2A (WRITE(10)), Flags=0x00 (no WRPROTECT/DPO/FUA)
    0x00, 0x00, 0x00, 0x02, // LBA: 0x00000000 (big-endian; changeable)
    0x00, // Group Number: 0
    0x00, // reserved
    0x01, 0x00, // Transfer Length: 1 block (big-endian 16-bit; changeable)
    0x00, // Control: 0x00
    0x00, 0x00, 0x00, 0x00, 
    0x00  // Padding (6 bytes to reach 31 total)
};

void SetMem(u8* data, u8* val, u16 size){
    u16 i = 0;
    for (i = 0; i < size; i++){
        data[i] = val[i];
    }
}

/* Print result */
void USB1_Print_String(u8 *data, u16 len, u8* string) {
    u8 tmp[100];
    u16 i = 0;

    if (data == NULL || len == 0)
        return;

    for (i = 0; i < len; i++) {
        tmp[i] = data[i];
    }
    tmp[len] = '\0'; // add null terminator

    printk("%s '%s'\n", string, tmp);
}

int USB1_Gather_LFN_String(u8 *data, u16 len, u8 *output_buf, u16 *buf_size) {
    u16 i = 0;
    u16 out_idx = 0;
    char *tmp = output_buf;
    int ret = 0;

    if (data == NULL || len == 0 || tmp == NULL) {
        *tmp = '\0';  // Null-terminate empty buffer
        return -1;
    }

    out_idx = len/2;
    for (i = 0; i < out_idx; i++){
        tmp[i] = (char)data[i*2];
        //printk("data[%d] = %c. - %c\n", i*2, data[i*2], tmp[i]);
        if (tmp[i] == 0x00){
            // stop here
            //printk("i = %d\n", i);
            ret = 1;
            break;
        }
    }

    *buf_size = i;
    tmp[*buf_size] = '\0';  // Null-terminate the string

    return ret;
}

void USB1_Print_Hex(u8 *data, u16 len, u8 *name)
{
    char line[3 * 8 + 1]; // "XX " * 8 bytes + null terminator = 25 chars
    u16 i;

    if (!data || len == 0)
        return;

    printk("# %s (len=%u bytes):\n", name, len);

    for (i = 0; i < len; i++) {
        int pos = (i % 8) * 3;
        snprintf(&line[pos], sizeof(line) - pos, "%02X ", data[i]);

        // Print every 8 bytes, or at the end of data
        if ((i % 8) == 7 || i == len - 1) {
            printk("  %s\n", line);
            memset(line, 0, sizeof(line));
        }
    }
}

void USB1_Print_HexVal(u8 *data, u16 len, u8 *name, bool little_endian){
    char str[128];
    int pos = 0;
    u16 i;

    if (!data || len == 0)
        return;

    printk("# %s (%s endian, len=%u): ", 
           name, little_endian ? "little" : "big", len);

    // Build hex string according to endianness
    if (little_endian) {
        for (i = 0; i < len; i++) // reverse order for printing
            pos += snprintf(str + pos, sizeof(str) - pos, "%02X", data[len - i - 1]);
    } else {
        for (i = 0; i < len; i++)
            pos += snprintf(str + pos, sizeof(str) - pos, "%02X", data[i]);
    }

    str[pos] = '\0';
    printk(" => 0x%s\n", str);
}

u32 USB1_Get_Bytes(u8 *data, u8 mode, bool little_endian){
    if (!data)
        return 0;

    switch (mode) {
    case 8:
        return data[0];

    case 16:
        if (little_endian)
            return get_unaligned_le16(data);
        else
            return get_unaligned_be16(data);

    case 32:
        if (little_endian)
            return get_unaligned_le32(data);
        else
            return get_unaligned_be32(data);

    default:
        return 0;
    }
}

void USB1_Get_String(u8* input, u8* output, u16 len){
    u16 i = 0;
    for (i = 0; i < len; i++){
        output[i] = input[i];
        if (input[i] == ' ') break;
    }
    output[i] = '\0';
}

int USB1_Compare_String(u8* input, u8* output, u16 len){
    u16 i = 0;
    for (i = 0; i < len; i++){
        if (output[i] != input[i]){
            return -1;
        }
    }
    return 0;

}

void USB1_Parse_TargetFile(u8* path, u8* output_dir, u32* output_dir_len, u8* output_file, u32* output_file_len) {
    u8* last_slash = NULL;
    u32 i;
    u32 path_len = 0;
    u32 dir_len = 0;
    u32 flen = 0;
    u8* file_start;
    *output_dir_len = 0;
    *output_file_len = 0;
    // Find the length of path
    while (path[path_len]) {
        path_len++;
    }
    // Find the last '/'
    for (i = 0; i < path_len; i++) {
        if (path[i] == '/') {
            last_slash = &path[i];
        }
    }
    if (last_slash == NULL) {
        // No slash found, set dir to "."
        output_dir[0] = '.';
        output_dir[1] = '\0';
        // Copy entire path as file
        memcpy(output_file, path, path_len);
        output_file[path_len] = '\0';
    } else {
        // Directory: up to but not including the last '/'
        dir_len = (u32)(last_slash - path);
        memcpy(output_dir, path, dir_len);
        output_dir[dir_len] = '\0';
        // File: after the last '/'
        file_start = last_slash + 1;
        flen = path_len - (u32)(file_start - path);
        memcpy(output_file, file_start, flen);
        output_file[flen] = '\0';
    }
    // Compute lengths
    while (output_dir[*output_dir_len] != '\0') {
        (*output_dir_len)++;
    }
    while (output_file[*output_file_len] != '\0') {
        (*output_file_len)++;
    }
}

void USB1_Parse_TargetDir(u8* path, u8* output_dir1, u32* output_dir1_len, u8* output_dir2, u32* output_dir2_len) {
    u8* last_slash = NULL;
    u32 i;
    u32 path_len = 0;
    u32 dir1_len = 0;
    u32 dir2_len = 0;
    *output_dir1_len = 0;
    *output_dir2_len = 0;

    // Find the length of path
    while (path[path_len]) {
        path_len++;
    }

    // Find the last '/'
    for (i = 0; i < path_len; i++) {
        if (path[i] == '/') {
            last_slash = &path[i];
        }
    }

    if (last_slash == NULL) {
        // No slash found, set dir1 to "./"
        output_dir1[0] = '.';
        output_dir1[1] = '/';
        output_dir1[2] = '\0';
        output_dir2[0] = '\0';
    } else {
        // dir1: up to but not including the last '/'
        dir1_len = (u32)(last_slash - path);
        memcpy(output_dir1, path, dir1_len);
        output_dir1[dir1_len] = '\0';

        // dir2: from the last '/' to end
        dir2_len = path_len - dir1_len;
        memcpy(output_dir2, last_slash, dir2_len);
        output_dir2[dir2_len] = '\0';
    }

    // Compute lengths
    while (output_dir1[*output_dir1_len] != '\0') {
        (*output_dir1_len)++;
    }
    while (output_dir2[*output_dir2_len] != '\0') {
        (*output_dir2_len)++;
    }
}

void USB1_Print_CSW(struct usb_device_data *data, u8* name){
    printk("# ----------------------------------- #\n");
    printk("# Status -> %s: \n", name);
    printk("# dCSWSignature = 0x%x\n", data->usb1_csw.dCSWSignature);
    printk("# dCSWTag = 0x%x\n", data->usb1_csw.dCSWTag);
    printk("# dCSWDataResidue = 0x%x\n", data->usb1_csw.dCSWDataResidue);
    printk("# bCSWStatus = 0x%x\n", data->usb1_csw.bCSWStatus);
    printk("# ----------------------------------- #\n");
}

void USB1_Print_SCSI_Inquiry(struct usb_device_data *data){
    u64 tmp[4];
    printk("# ------------- USB_INFO ------------ #\n");
    printk("# additional_length = 0x%x\n", data->scsi_inquiry.additional_length);
    USB1_Print_String(data->scsi_inquiry.vendor_id, 8, "# vendor_id");
    USB1_Print_String(data->scsi_inquiry.product_id, 16, "# product_id");
    USB1_Print_String(data->scsi_inquiry.product_revision, 4, "# product_revision");

    data->scsi_inquiry.LBA = swap_endian32(data->scsi_inquiry.LBA);
    data->scsi_inquiry.Capacity = swap_endian32(data->scsi_inquiry.Capacity);
    //printk("# Logical block address(LBA) = 0x%x\n", data->scsi_inquiry.LBA);
    printk("# block size = 0x%x\n", data->scsi_inquiry.Capacity);

    tmp[0] = data->scsi_inquiry.LBA;
    tmp[1] = data->scsi_inquiry.Capacity;
    tmp[2] = tmp[0]*tmp[1];
    tmp[3] = div_u64(tmp[2], 1000000000ULL);
    printk("# => Total = %lld bytes | ~ %lld GB\n", tmp[2], tmp[3] + 1);
    printk("# ----------------------------------- #\n");
}

void USB1_Print_EAA_Instance(struct usb_device_data *data, u8* name){
    printk("# ----------------------------------- #\n");
    printk("# %s: \n", name);
    printk("# Mode_data_len = 0x%x\n", cbw_EAA_Instance.Mode_data_len);
    printk("# Reserved1 = 0x%x\n", cbw_EAA_Instance.Reserved1);
    printk("# Block_descriptor_len = 0x%x\n", cbw_EAA_Instance.Block_descriptor_len);

    if (cbw_EAA_Instance.Mode_data_len > 3){
        USB1_Print_Hex(cbw_EAA_Instance.block_descriptors, 5, "block_descriptors");
        USB1_Print_Hex(cbw_EAA_Instance.mode_pages, 61, "mode_pages");
    }
    printk("# ----------------------------------- #\n");
}

/* Utils */
void USB1_Clear_CBW(struct usb_device_data *data){
    u16 i = 0;
    u8* ptr = (u8*)&data->usb1_cbw;
    u16 size = sizeof(data->usb1_cbw);

    for (i = 0; i < size; i++){
        ptr[i] = 0;
    }
}
void USB1_Set_CBW(struct usb_device_data *data, const u8* d){
    u16 i = 0;
    u8* ptr = (u8*)&data->usb1_cbw;
    u16 size = sizeof(data->usb1_cbw);

    for (i = 0; i < size; i++){
        ptr[i] = d[i];
    }
}
void USB1_Apply_CBW(struct usb_device_data *data, const u8* d){
    USB1_Clear_CBW(data);
    USB1_Set_CBW(data, d);
    data->usb1_cbw.dCBWTag++; // tag will be automatically updated
}

/* ================== API for HMSC bulk Transfer ===================== */
int USB1_Send_INQUIRY(struct usb_device_data *data, u8* cbw, u8* data_inquiry, bool print_status, u8* name){
    int ret;
    u16 InDataLen;

    // 1. Command: Bulk OUT CBW (31 bytes)
    USB1_Apply_CBW(data, cbw);
    ret = USB1_OUT_Phase_Bulk(data, 0x2, Global_Address, (u8*)&data->usb1_cbw, sizeof(data->usb1_cbw));
    if (ret < 0) {
        printk("%s: CBW OUT failed: %d\n", name, ret);
        return ret;
    }

    if (data_inquiry){
        // 2. Data: Bulk IN (36 bytes)
        ret = USB1_IN_Phase_Bulk(data, 0x1, Global_Address, data_inquiry, &InDataLen);
        if (ret < 0) {
            printk("%s: DATA IN failed: %d (len=%d)\n", name, ret, InDataLen);
            return ret;
        }
        else {
#if (PRINT_DEBUG)
            printk("InDataLen = %d\n", InDataLen);
#endif
        }
    }

    // 3. Data: Bulk IN CSW (16 bytes)
    ret = USB1_IN_Phase_Bulk(data, 0x1, Global_Address, (u8*)&data->usb1_csw, &InDataLen);
    if (ret < 0) {
        printk("%s: CSW IN failed: %d (len=%d)\n", name, ret, InDataLen);
        return ret;
    }
    else {
        if (print_status) USB1_Print_CSW(data, name);
    }


    return 0;
}

int USB1_CBW(struct usb_device_data *data){
    int ret;

    ret = USB1_Send_INQUIRY(data, cbw_initial, (u8*)&data->scsi_inquiry, 0, "CBW Initial");
    msleep(500);

    if (ret == 0){
        ret = USB1_Send_INQUIRY(data, cbw_capacity, (u8*)&data->scsi_inquiry + 0x24, 0, "CBW Capacity");
    }

    // cbw_EAA_Instance
    if (ret == 0){
        ret = USB1_Send_INQUIRY(data, cbw_mode_sense_EAA, (u8*)&cbw_EAA_Instance, 0, "cbw_EAA_Instance");
    }

    // cbw_EAA_Instance
    if (ret == 0){
        ret = USB1_Send_INQUIRY(data, cbw_test_ready, NULL, 0, "cbw_test_ready");
    }

    /* Print result */
    if (ret == 0){
#if (PRINT_USB_INFO)
        USB1_Print_SCSI_Inquiry(data);
        USB1_Print_EAA_Instance(data, "cbw_EAA_Instance");
#endif
    }

    return ret;
}

/* ================== API for HMSC bulk read Transfer ===================== */
int USB1_Read_SECTOR(struct usb_device_data *data, u32 LBA, u16 block_size, u8* sector_data, u16* sector_data_len, u8* name, u8 print_status){
    int ret;
    u16 CSWDataLen;

    USB1_Apply_CBW(data, cbw_read_sector);
    SetMem((u8*)&data->usb1_cbw.CBWCB[2], (u8*)&LBA, 4); // 4 (2-5) bytes for LBA
    SetMem((u8*)&data->usb1_cbw.CBWCB[8], (u8*)&block_size, 2); // 2 (8-9) bytes for block size


    // // convert big endian for LBA
    swap_endian32Ptr((u32*)&data->usb1_cbw.CBWCB[2]);


    // USB1_Print_Hex((u8*)&data->usb1_cbw.CBWCB[2], 4, "LBA");
    // USB1_Print_Hex((u8*)&data->usb1_cbw.CBWCB[8], 2, "block size");

    // USB1_Print_Hex((u8*)&data->usb1_cbw.dCBWSignature, 31, "cbw");

    // 1. Command: Bulk OUT CBW (31 bytes)
    ret = USB1_OUT_Phase_Bulk(data, 0x2, Global_Address, (u8*)&data->usb1_cbw, sizeof(data->usb1_cbw));
    if (ret < 0) {
        printk("%s: CBW OUT failed: %d\n", name, ret);
        return ret;
    }

    if (sector_data){
        // 2. Data: Bulk IN (36 bytes)
        ret = USB1_IN_Phase_Bulk(data, 0x1, Global_Address, sector_data, sector_data_len);
        if (ret < 0) {
            printk("%s: DATA IN failed: %d (len=%d)\n", name, ret, *sector_data_len);
            return ret;
        }
        else {
        //printk("sector_data_len = %d\n", *sector_data_len);
        }
    }

    // 3. Data: Bulk IN CSW (16 bytes)
    ret = USB1_IN_Phase_Bulk(data, 0x1, Global_Address, (u8*)&data->usb1_csw, &CSWDataLen);
    if (ret < 0) {
        printk("%s: CSW IN failed: %d (len=%d)\n", name, ret, CSWDataLen);
        return ret;
    }
    else {
        if (print_status) USB1_Print_CSW(data, name);
    }

    return 0; 
}

int USB1_Read_SECTOR_DATA(struct usb_device_data *data, u32 LBA, u16 block_size, u8* sector_data, u16* sector_data_len, u8* name, u8 print_status, u8 print_data){
    int ret;

    ret = USB1_Read_SECTOR(data, LBA, block_size, sector_data, sector_data_len, name, print_status);
    if (ret < 0) {
        printk("Fail USB1_Read_SECTOR\n");
        return -1;
    }
    else {
        if (print_data) USB1_Print_Hex(sector_data, 512, name);
    }

    return 0;
}

/* ================== API for HMSC bulk write Transfer ===================== */
int USB1_Write_SECTOR(struct usb_device_data *data, u32 LBA, u16 block_size, u8* sector_data, u16 sector_data_len, u8* name, u8 print_status){
    int ret;
    u16 CSWDataLen;

    USB1_Apply_CBW(data, cbw_write_sector);
    SetMem((u8*)&data->usb1_cbw.CBWCB[2], (u8*)&LBA, 4); // 4 (2-5) bytes for LBA
    SetMem((u8*)&data->usb1_cbw.CBWCB[8], (u8*)&block_size, 2); // 2 (8-9) bytes for block size


    // // convert big endian for LBA
    swap_endian32Ptr((u32*)&data->usb1_cbw.CBWCB[2]);


    // USB1_Print_Hex((u8*)&data->usb1_cbw.CBWCB[2], 4, "LBA");
    // USB1_Print_Hex((u8*)&data->usb1_cbw.CBWCB[8], 2, "block size");

    // USB1_Print_Hex((u8*)&data->usb1_cbw.dCBWSignature, 31, "cbw");

    // 1. Command: Bulk OUT CBW (31 bytes)
    ret = USB1_OUT_Phase_Bulk(data, 0x2, Global_Address, (u8*)&data->usb1_cbw, sizeof(data->usb1_cbw));
    if (ret < 0) {
        printk("%s: CBW OUT failed: %d\n", name, ret);
        return ret;
    }

    if (sector_data){
        // 2. Data: Bulk OUT (512 bytes)
        ret = USB1_OUT_Phase_Bulk(data, 0x2, Global_Address, sector_data, sector_data_len);
        if (ret < 0) {
            printk("%s: DATA OUT failed: %d (len=%d)\n", name, ret, sector_data_len);
            return ret;
        }
        else {
        //printk("sector_data_len = %d\n", *sector_data_len);
        }
    }

    // 3. Data: Bulk IN CSW (16 bytes)
    ret = USB1_IN_Phase_Bulk(data, 0x1, Global_Address, (u8*)&data->usb1_csw, &CSWDataLen);
    if (ret < 0) {
        printk("%s: CSW IN failed: %d (len=%d)\n", name, ret, CSWDataLen);
        return ret;
    }
    else {
        if (print_status) USB1_Print_CSW(data, name);
    }

    return 0; 
}

int USB1_Write_SECTOR_DATA(struct usb_device_data *data, u32 LBA, u16 block_size, u8* sector_data, u16 sector_data_len, u8* name, u8 print_status, u8 print_data){
    int ret;

    ret = USB1_Write_SECTOR(data, LBA, block_size, sector_data, sector_data_len, name, print_status);
    if (ret < 0) {
        printk("Fail USB1_Write_SECTOR\n");
        return -1;
    }
    else {
        if (print_data) USB1_Print_Hex(sector_data, 512, name);
    }

    return 0;
}
