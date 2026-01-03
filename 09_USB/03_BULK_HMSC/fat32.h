#ifndef F_H
#define F_H

#include "usb1.h"
#include "mass_storage.h"
#include <linux/math64.h>

/*
    - DIR SFN only (except Hidden DIR) the dir entry is located at the third index (after the self + parent reference index)
    - Hidden DIR SFN doesn't have '.' at the start while Hidden DIR LFN has
*/

/* Macro value */
#define FILE_TYPE 0x20
#define DIR_TYPE 0x10
#define LFN_TYPE 0x0F

#define CONTENT_TYPE 0x21

/* Macro offser */
#define SEQ_NUM 0x1F /* 5 bits */
#define END_MARKER 0x40 /* bit 6 */

/* Dir size */
#define MAX_DIRS 100
#define MAX_PATH_LEN 256
#define MAX_FULL_PATH_LEN (MAX_PATH_LEN - 3)

#define PRINT_DIR 1
#define NO_PRINT_DIR 0

#define ALL_DIR_READ_ALL 1u << 2
#define ALL_DIR 1
#define CURRENT_DIR 0

#define PRINT_DATA 1
#define NO_PRINT_DATA 0

// Forward declaration for function parameters
struct usb_device_data;

/* BIOS Parameter Block (BPB) Details */
typedef struct BPB {  
    u16 Starting_LBA;
    u8 OME_name[8];
    u8 Partition_type;
    u16 Bytes_per_Sector;
    u8 Sectors_per_Cluster;
    u16 Reserved_Sectors;
    u16 Number_of_FATs;
    u32 Sectors_per_FAT;
    u8 FS_type[8];
    u32 Root_Cluster;
    u32 Data_Sector;
} BPB;

/* FAT Root Directory Entry */
typedef struct Root_Directory_Entry {  
    u8 id;
    u8 File_name[10];
    u8 File_attributes;
    u8 Reserved;
    u8 File_create_time;
    u16 Create_time;
    u16 Create_date;
    u16 Access_date;
    u16 High_first_cluster;
    u16 Modified_time;
    u16 Modified_date;
    u16 Low_first_cluster;
    u32 File_size; // 0 for directory
} Root_Dir_Entry;

/* LFN Directory Entry */
typedef struct LFN_Root_Directory_Entry {  
    u8 id;
    u8 File_name1[10];
    u8 File_attributes;
    u8 type; // 0x00 -> lowcase
    u8 checksum;  // Checksum of corresponding short (8.3) filename
    u8 File_name2[12];
    u16 Reserved;
    u8 File_name3[4];
} LFN_Root_Dir_Entry;

int USB1_Read_CLUSTER(struct usb_device_data *data, u32 cluster_num, u8* cluster_data, u32* cluster_data_len, u8* name, bool print_data);
int USB1_Write_CLUSTER(struct usb_device_data *data, u32 cluster_num, u8* cluster_data, u8* name, bool print_data);

/* FAT32 driver */
int USB1_f_Mount(struct usb_device_data *data);
int USB1_f_Read_All(struct usb_device_data *data);
int USB1_f_Read_Dir(struct usb_device_data *data, u8* path_dir);
int USB1_f_Read_File(struct usb_device_data *data, u8* path_file);
int USB1_f_Make_Dir(struct usb_device_data *data, u8* path_dir);
int USB1_f_Remove_Dir(struct usb_device_data *data, u8* path_dir);
int USB1_f_Make_File(struct usb_device_data *data, u8* path_file);
int USB1_f_Remove_File(struct usb_device_data *data, u8* path_dir);
int USB1_f_Add_Content(struct usb_device_data *data, u8* path_file, u8* content);
int USB1_f_Delete_Content(struct usb_device_data *data, u8* path_file);

#endif /* F_H */
