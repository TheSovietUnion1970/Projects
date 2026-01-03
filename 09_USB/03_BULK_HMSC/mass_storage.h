#ifndef MS_H
#define MS_H

#define SECTOR_SIZE 512
#define LITTLE_ENDIAN 1
#define BIG_ENDIAN 1


// Forward declaration for function parameters
struct usb_device_data;

/* SCSI command */
typedef struct __attribute__((packed)) usb_msc_cbw {  // Command Block Wrapper
    u32 dCBWSignature;    // 0x43425355 ("USBC")
    u32 dCBWTag;          // Unique tag (increment per command)
    u32 dCBWDataTransferLength;  // 36 for INQUIRY
    u8 bmCBWFlags;        // 0x80 (IN direction)
    u8 bCBWLUN;           // 0 (LUN 0)
    u8 bCBWCBLength;      // 6 (SCSI cmd length)
    u8 CBWCB[16];         // SCSI INQUIRY: {0x12, 0x00, 0x00, 0x00, 36, 0, ...}
} msc_cbw;

typedef struct __attribute__((packed)) usb_msc_csw {  // Command Status Wrapper
    u32 dCSWSignature;    // 0x53425355 ("USBS")
    u32 dCSWTag;          // Matches CBW tag
    u32 dCSWDataResidue;  // Untransferred bytes (0 for success)
    u8 bCSWStatus;        // 0x00=Passed, 0x01=Failed, 0x02=Phase Error
} msc_csw;

/* SCSI data structure */
typedef struct __attribute__((packed)) scsi_inquiry_response {
    // 0-30
    u8 peripheral_qualifier : 3;  // Bits 7-5: Qualifier (0=connected)
    u8 peripheral_device_type : 5;  // Bits 4-0: Type (0x00=direct-access block)
    u8 rmb : 1;  // Bit 7: Removable Media Bit (1=removable)
    u8 reserved1 : 7;  // Bits 6-0: Reserved
    u8 version;  // ANSI/ISO version (0x00=basic compliance)
    u8 response_data_format : 4;  // Bits 3-0: Format (0x01=SCSI-1 style)
    u8 reserved2 : 4;  // Bits 7-4: Reserved (includes AERC, Obsolete, NormACA, HiSup)
    u8 additional_length;  // Length of remaining data (0x1F=31 bytes, total 36)
    u8 sccs : 1;  // Bit 7: SCC Supported
    u8 addr16 : 1;  // Bit 6: 16-bit wide SCSI addresses
    u8 reserved3 : 6;  // Bits 5-0: Includes MChngr, MultiP, EncServ, etc.
    u8 reserved4;  // Byte 6: More flags (e.g., RelAdr, WBus32)
    u8 soft_reset : 1;  // Bit 7: Soft Reset support
    u8 cmdque : 1;  // Bit 6: Command Queuing
    u8 reserved5 : 6;  // Bits 5-0: Includes Linked, Sync
    char vendor_id[8];  // ASCII, space-padded: "Mass    "
    char product_id[16];  // ASCII, space-padded: "Storage Device  "
    char product_revision[4];  // ASCII: "1.00"

    // 31-38
    u32 LBA;
    u32 Capacity;
} inquiry_response;

/* Element Address Assignment Page */
typedef struct cbw_EAA {  // Command Status Wrapper
    u8 Mode_data_len;
    u16 Reserved1;
    u8 Block_descriptor_len;

    u8 block_descriptors[5];         // Bytes 4-8: Invalid/short descriptors (ignore or pad to 8)
    u8 mode_pages[61];               // Bytes 9-69: Mode pages (e.g., 0x3F all pages; total ~60 bytes usable)
} cbw_EAA;

/* Print result */
void USB1_Print_String(u8 *data, u16 len, u8* string);
void USB1_Print_Hex(u8 *data, u16 len, u8* name);
void USB1_Print_HexVal(u8 *data, u16 len, u8 *name, bool little_endian);
int USB1_Gather_LFN_String(u8 *data, u16 len, u8 *output_buf, u16 *buf_size);
void USB1_Get_String(u8* input, u8* output, u16 len);
int USB1_Compare_String(u8* input, u8* output, u16 len);
void USB1_Parse_TargetFile(u8* path, u8* output_dir, u32* output_dir_len, u8* output_file, u32* output_file_len);
void USB1_Parse_TargetDir(u8* path, u8* output_dir1, u32* output_dir1_len, u8* output_dir2, u32* output_dir2_len);
u32 USB1_Get_Bytes(u8 *data, u8 mode, bool little_endian);

/* ================== API for HMSC Bulk Transfer ===================== */ 
int USB1_Send_INQUIRY(struct usb_device_data *data, u8* cbw, u8* data_inquiry, bool print_status, u8* name);

int USB1_CBW(struct usb_device_data *data);

int USB1_Read_SECTOR(struct usb_device_data *data, u32 LBA, u16 block_size, u8* sector_data, u16* sector_data_len, u8* name, u8 print_status);
int USB1_Read_SECTOR_DATA(struct usb_device_data *data, u32 LBA, u16 block_size, u8* sector_data, u16* sector_data_len, u8* name, u8 print_status, u8 print_data);

int USB1_Write_SECTOR(struct usb_device_data *data, u32 LBA, u16 block_size, u8* sector_data, u16 sector_data_len, u8* name, u8 print_status);
int USB1_Write_SECTOR_DATA(struct usb_device_data *data, u32 LBA, u16 block_size, u8* sector_data, u16 sector_data_len, u8* name, u8 print_status, u8 print_data);

#endif /* MS_H */
