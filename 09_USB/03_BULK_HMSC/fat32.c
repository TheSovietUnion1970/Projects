#include "fat32.h"
#include "main.h"

BPB bpb_instance;
Root_Dir_Entry rde_instance;

/* Global buffer for reading content of file / directory */
u8 Cluster_data[8][SECTOR_SIZE];

/* Global buffer for mounting */
u8 Glob_Sector_data[SECTOR_SIZE];

/* Global buffer for next cluster */
u8 Next_cluster_data[8][SECTOR_SIZE];

/* Global padding index */
u8 padding_index = 0;

/* Global array for holding dir name and dir cluster */
u8 names[MAX_DIRS][MAX_PATH_LEN]; // up to 10 directories
u32 Dir_cluster[MAX_DIRS];
u32 Dir_padding[MAX_DIRS];
u8 Dir_index = 0;

int num_direct[MAX_DIRS];
bool all_children_leaves[MAX_DIRS];
int max_subtree_padding[MAX_DIRS];
int parent[MAX_DIRS];
bool include_flags[MAX_DIRS];
int chain_indices[MAX_DIRS];

char path[MAX_FULL_PATH_LEN];
char stack[MAX_DIRS][MAX_PATH_LEN];
char full_paths[MAX_DIRS][MAX_FULL_PATH_LEN];
char prefix[MAX_PATH_LEN + 2];
char actual_dirs[MAX_DIRS][MAX_PATH_LEN];

Root_Dir_Entry Ins_entry[MAX_DIRS];

int USB1_Scan_Cluster(struct usb_device_data *data, u8* cluster_data, u8 cluster_num, bool print_data, u8 all_dir);

int USB1_Read_CLUSTER(struct usb_device_data *data, u32 cluster_num, u8* cluster_data, u32* cluster_data_len, u8* name, bool print_data){
    int ret;
    u8 i = 0;
    u16 Sector_data_len;
    u32 sector_num;
    *cluster_data_len = 0;

    /* Caculate sector num from setor num */
    sector_num = bpb_instance.Starting_LBA + bpb_instance.Data_Sector + (cluster_num - 2)*bpb_instance.Sectors_per_Cluster;

    for (i = 0; i < 8; i++){
        ret = USB1_Read_SECTOR_DATA(data, sector_num + i, 1, (u8*)(cluster_data + i*SECTOR_SIZE), &Sector_data_len, "Sector N", 0, print_data);
        //printk("Addr: = 0x%x\n", (u8*)&Cluster_data[i][0]);
        if (ret < 0){
            printk("Fail at index: %d\n", i);
            return -1;
        }

        *cluster_data_len += SECTOR_SIZE;
    }

    return 0;
}

int USB1_Write_CLUSTER(struct usb_device_data *data, u32 cluster_num, u8* cluster_data, u8* name, bool print_data){
    int ret;
    u8 i = 0;
    u32 sector_num;
    u32 sector_data_len = SECTOR_SIZE;

    /* Caculate sector num from setor num */
    sector_num = bpb_instance.Starting_LBA + bpb_instance.Data_Sector + (cluster_num - 2)*bpb_instance.Sectors_per_Cluster;

    for (i = 0; i < 8; i++){
        ret = USB1_Write_SECTOR_DATA(data, sector_num + i, 1, (u8*)(cluster_data + i*SECTOR_SIZE), sector_data_len, "Sector N", 0, print_data);
        //printk("Addr: = 0x%x\n", (u8*)&Cluster_data[i][0]);
        if (ret < 0){
            printk("Fail at index: %d\n", i);
            return -1;
        }

        //*cluster_data_len += SECTOR_SIZE;
    }

    return 0;
}

void CpyMemff(u8* dst, u8* src, u16 size){
    u16 i = 0;
    for (i = 0; i < size; i++){
        dst[i] = src[i];
    }
}

void SetMemff(u8* src, u8 val, u16 size){
    u16 i = 0;
    for (i = 0; i < size; i++){
        src[i] = val;
    }
}

void get_tree_padding(int padding_index, u8* buf, u8 type_file) {
    int i;

    if (padding_index < 0) {
        strcpy(buf, "");  // Invalid: empty
        //return buf;
        return;
    }

    // Clear buffer
    memset(buf, 0, 200);

    // Add (padding_index) spaces
    for (i = 0; i < padding_index*3; i++) {
        buf[i] = ' ';
    }

    // Append "|-"
    if (type_file == FILE_TYPE) strcpy(buf + i, "|----\0");
    else if (type_file == CONTENT_TYPE) strcpy(buf + i, "|-[.]\0");
    else strcpy(buf + i, "|-[ ]\0");
}

/* SFN_entry is a pointer to SFN dir entry */
void USB1_Read_NameFile(Root_Dir_Entry* SFN_entry, u8* Output_name, u16* Output_len){
    Root_Dir_Entry* entry_start;
    u16 tmp_len;
    int LFN_ret;

    *Output_len = 0;

    entry_start = SFN_entry;

    // =============== checking the LFN name
    entry_start -= 1;

    while(!((entry_start->id)&END_MARKER)){
        LFN_ret = USB1_Gather_LFN_String(((u8*)entry_start) + 1, 10, Output_name + *Output_len, &tmp_len);
        *Output_len += tmp_len;

        if (LFN_ret == 0){
            LFN_ret = USB1_Gather_LFN_String(((u8*)entry_start) + 14, 12, Output_name + *Output_len, &tmp_len);
            *Output_len += tmp_len;
        }

        if (LFN_ret == 0){
            LFN_ret = USB1_Gather_LFN_String(((u8*)entry_start) + 28, 4, Output_name + *Output_len, &tmp_len);
            *Output_len += tmp_len;
        }

        entry_start-= 1; // back the previous entry
    }

    if ((entry_start->id)&END_MARKER){
            //printk("hehe XXXXXXXXXXXXX\n");
        LFN_ret = USB1_Gather_LFN_String(((u8*)entry_start) + 1, 10, Output_name + *Output_len, &tmp_len);
        *Output_len += tmp_len;
        
        if (LFN_ret == 0){
            LFN_ret = USB1_Gather_LFN_String(((u8*)entry_start) + 14, 12, Output_name + *Output_len, &tmp_len);
            *Output_len += tmp_len;
            //USB1_Print_String(tmp_Root_Dir_Entry + 14, 12, "File name 2s:");
        }      
        if (LFN_ret == 0){
            LFN_ret = USB1_Gather_LFN_String(((u8*)entry_start) + 28, 4, Output_name + *Output_len, &tmp_len);
            *Output_len += tmp_len;
        }   
    }
}

void USB1_Scan_ClusterData(struct usb_device_data *data, u8* cluster_data, u8 cluster_num, bool print_dir, u8 all_dir){
    u32 cluster_len = (SECTOR_SIZE*8)/32;
    u32 i = 0, Cluster_data_len = 0;
    int ret = 0;
    u8* tmp_ptr = cluster_data;
    u8 tmp_file_name[50];
    u16 tmp_file_name_len = 0;

    u8* ptr8;
    bool hidden_folder = false;

    Root_Dir_Entry* valid_Dir_Entry[100];
    u8 valid_Dir_Entry_index = 0;

    /* Local buffer for padding */
    u8 padding_buffer[200];

    // make 2D into 1D
    for (i = 0; i < cluster_len; i++){
        if ((tmp_ptr[i*32] == 0xE5U)) // deleted or unallocated file
        {
            // do nothing as notthing should be shown
        }
        else if (tmp_ptr[i*32] == 0x00U){
            break; // as no entry anymore
        }
        else // the name of read data
        {
            //printk("i = %d\n", i);
            if (tmp_ptr[i*32 + 11] == FILE_TYPE) {
                valid_Dir_Entry[valid_Dir_Entry_index++] = (Root_Dir_Entry*)(cluster_data + i*32); // save ptr to valid root dir entry
                //printk("index FILE = %d\n", i*32);
            }
            else if ((tmp_ptr[i*32 + 11] == LFN_TYPE) && ((tmp_ptr[i*32 + 0])&SEQ_NUM) == 0x1) { // get the first first entry of LFN
                valid_Dir_Entry[valid_Dir_Entry_index++] = (Root_Dir_Entry*)(cluster_data + i*32); // save ptr to valid root dir entry

                //printk("index = %d\n", i);
            }
            else if (tmp_ptr[i*32 + 11] == DIR_TYPE) {
                valid_Dir_Entry[valid_Dir_Entry_index++] = (Root_Dir_Entry*)(cluster_data + i*32); // save ptr to valid root dir entry
            }
        }
    }

    //if (print_data){
    for (i = 0; i < valid_Dir_Entry_index; i++){
        if (!valid_Dir_Entry[i]) {
            printk("valid_Dir_Entry is NULL");
            break;
        }
        else {
#if (PRINT_CONTENT)
            // content of file
            if (valid_Dir_Entry[i]->File_attributes == FILE_TYPE) { 
                ret = USB1_Read_CLUSTER(data, (valid_Dir_Entry[i]->High_first_cluster << 16) | (valid_Dir_Entry[i]->Low_first_cluster), (u8*)Cluster_data, &Cluster_data_len, "Cluster next", 0);
                if ((ret == 0)){
                    get_tree_padding(padding_index + 1, padding_buffer, CONTENT_TYPE); 
                    if (all_dir&ALL_DIR_READ_ALL) USB1_Print_String((u8*)Cluster_data, valid_Dir_Entry[i]->File_size, padding_buffer);
                }       
            }
#endif
            // Name of file (SFN + LFN) / dir (LFN)
            if ((valid_Dir_Entry[i]->File_attributes == LFN_TYPE) && (((valid_Dir_Entry[i]->id)&SEQ_NUM) == 0x01)) {
                USB1_Read_NameFile(valid_Dir_Entry[i] + 1, tmp_file_name, &tmp_file_name_len);

                // checking whether file or folder by heading 1 dir to check file attributes
                if (valid_Dir_Entry[i + 1]->File_attributes == FILE_TYPE) {

                    get_tree_padding(padding_index, padding_buffer, FILE_TYPE); 

                    //printk("all_dir1 = 0x%x\n", all_dir);
                    if (print_dir && (all_dir&ALL_DIR_READ_ALL)) USB1_Print_String(tmp_file_name, tmp_file_name_len, padding_buffer);
                }
                else {
#if (HIDDEN_FOLDERS)
                    get_tree_padding(padding_index, padding_buffer, DIR_TYPE); 
                    //printk("all_dir2 = 0x%x\n", all_dir);
                    if (all_dir&ALL_DIR_READ_ALL) USB1_Print_String(tmp_file_name, tmp_file_name_len, padding_buffer);

                    /* Save Dir name into buffer */
                    USB1_Get_String(tmp_file_name, names[Dir_index], tmp_file_name_len);
                    Dir_padding[Dir_index] = padding_index;
#else
                    /* Checking hidden folders */
                    // as Hidden LFN Dir always has '.' at the start
                    if (tmp_file_name[0] != '.'){
                        get_tree_padding(padding_index, padding_buffer, DIR_TYPE); 
                        if (print_dir && (all_dir&ALL_DIR_READ_ALL)) USB1_Print_String(tmp_file_name, tmp_file_name_len, padding_buffer);

                        /* Save Dir name into buffer */
                        // (valid_Dir_Entry[i]->High_first_cluster << 16) | (valid_Dir_Entry[i]->Low_first_cluster)
                        USB1_Get_String(tmp_file_name, names[Dir_index], tmp_file_name_len);
                        Dir_padding[Dir_index] = padding_index;
                    }
#endif
                } 
            }
        
            // Name of dir (SFN) only else the name will be printed in the above if
            // valid_Dir_Entry[i]->id != '.' to prevent cluster N 0x2E at index 0 and 1
            else if (valid_Dir_Entry[i]->File_attributes == DIR_TYPE && valid_Dir_Entry[i]->id != '.'){
                if (ret == 0){
                    u8 next_cluster = 0;
                    next_cluster = (valid_Dir_Entry[i]->High_first_cluster << 16) | (valid_Dir_Entry[i]->Low_first_cluster);

#if (!HIDDEN_FOLDERS)
                    /* DIR SFN only (except Hidden DIR) when dir entry is located at the third index of cluster (self + parent index), 
                        DIR SFN + LFN when dir entry is not located at the third index of cluster,
                        When not in root cluster (0x2), cluster starts with 2 dir entries (self + parent) */
                    ptr8 = (u8*)valid_Dir_Entry[i];
                    ptr8-=32;
                    /* Back the previous dir entry to check hidden folder ('.')
                    and make sure dir entry is not located at the first index of cluster (i != 0) */
                    if ((((ptr8[0])&SEQ_NUM) == 0x01) && (ptr8[1] == '.') && (i != 0)){
                        //printk("HIDDEN FILE\n");
                        hidden_folder = true;
                    }
#endif
                    // avoid printing SFN Dir as LFN Dir will be printed or only SFN Dir is printed
                    ptr8 = (u8*)valid_Dir_Entry[i];
                    ptr8-=32; // back to the previous entry to check LFN

                    get_tree_padding(padding_index, padding_buffer, DIR_TYPE); 
                    //printk("all_dir3 = 0x%x\n", all_dir);
                    if (ptr8[11] != 0x0F){ // print SFN only when LFN is non-existent
                        if (print_dir && (all_dir&ALL_DIR_READ_ALL)) USB1_Print_String((u8*)valid_Dir_Entry[i], 11, padding_buffer);

                        /* Save Dir name into buffer */
                        USB1_Get_String((u8*)valid_Dir_Entry[i], names[Dir_index], 11);
                        Dir_padding[Dir_index] = padding_index;
                    }

                    // only get next_cluster when SFN DIR
                    Dir_cluster[Dir_index+1] = (valid_Dir_Entry[i]->High_first_cluster << 16) | (valid_Dir_Entry[i]->Low_first_cluster);
                    Dir_index++;

                    padding_index++;
                    if (all_dir && !hidden_folder) {
                        //printk("XXX\n");
                        ret = USB1_Scan_Cluster(data, (u8*)&Next_cluster_data, next_cluster, print_dir, all_dir);
                    }
                    padding_index--;
                    hidden_folder = false;

                    /* Reading dir entry again for current_index_index use to optimize stack size */
                    if (ret == 0){
                        ret = USB1_Read_CLUSTER(data, cluster_num, (u8*)Next_cluster_data, &Cluster_data_len, "Reading dir entry again", 0);
                    }
                }
            }
        }
        
    }
    //}
}

int USB1_Scan_Cluster(struct usb_device_data *data, u8* cluster_data, u8 cluster_num, bool print_dir, u8 all_dir){
    int ret;
    u32 Cluster_data_len = 0;
    u32* ptr32;

    /* Local buffer for checking FAT */
    u8 FAT_Sector_data[SECTOR_SIZE];
    u16 FAT_Sector_data_len = 0;

    /* Read cluster num for root directory entry */
    ret = USB1_Read_CLUSTER(data, cluster_num, (u8*)cluster_data, &Cluster_data_len, "Reading dir entry", NO_PRINT_DATA);

    //printk(">>> === [cluster %d starts] === \n", cluster_num);
    // /* Scan root dir */
    if (ret == 0){
        USB1_Scan_ClusterData(data, (u8*)cluster_data, cluster_num, print_dir, all_dir);
    }

    /* Check FAT */
    if (ret == 0){
        ret = USB1_Read_SECTOR_DATA(data, bpb_instance.Starting_LBA + bpb_instance.Reserved_Sectors, 1, FAT_Sector_data, &FAT_Sector_data_len, "Sector starting LBA", 0, 0);
        if (ret == 0){
            ptr32 = (u32*)FAT_Sector_data;

            /* Check if end of cluster */
            if (ptr32[cluster_num] == 0x0FFFFFFF){
                //printk(" === [cluster %d ends] === <<<\n", cluster_num);
            }
            else {
                //printk("cluster %d doesn not end -> next: 0x%x\n", cluster_num, ptr32[cluster_num]);
            }
        }
    }

    return ret;
}

int USB1_f_Mount(struct usb_device_data *data){
    int ret;
    u16 Sector_data_len;
    u32 partition_entry_offset = 0x01BE;

    /* Read sector 0 */
    ret = USB1_Read_SECTOR_DATA(data, 0x0, 1, Glob_Sector_data, &Sector_data_len, "Sector 0", 0, PRINT_DATA);
    if (ret == 0){
        USB1_Get_String(Glob_Sector_data + 3, bpb_instance.OME_name, 8);
#if (PRINT_DEBUG)
        USB1_Print_String(Glob_Sector_data + 3, 8, "OEM name");
#endif

        /* Check MBR or VBR with valid OME name */
        if ((USB1_Compare_String("mkfs.fat", bpb_instance.OME_name, strlen("mkfs.fat")) == 0) ||
            (USB1_Compare_String("NTFS", bpb_instance.OME_name, strlen("NTFS")) == 0)){
            printk("THis sector is for VBR\n");
            bpb_instance.Starting_LBA = 0; // set this value to 0 to let USB1_Read_SECTOR_DATA below to read at sector 0
        }
        else {
            printk("THis sector is for MBR, need to check partition entry\n");
            bpb_instance.Starting_LBA = USB1_Get_Bytes(Glob_Sector_data + partition_entry_offset + 0x08, 32, LITTLE_ENDIAN);
            bpb_instance.Partition_type = USB1_Get_Bytes(Glob_Sector_data + partition_entry_offset + 0x04, 8, LITTLE_ENDIAN);

            USB1_Print_HexVal(Glob_Sector_data + partition_entry_offset + 0x04, 1, "Partition type", LITTLE_ENDIAN);
            USB1_Print_HexVal(Glob_Sector_data + partition_entry_offset + 0x08, 4, "Starting LBA", LITTLE_ENDIAN);
            USB1_Print_HexVal(Glob_Sector_data + partition_entry_offset + 0x0C, 4, "Num of sectors in partition", LITTLE_ENDIAN);
            /* .. do later ... */
        }
    }

    /* Read sector at the starting LBA */
    if (ret == 0){
        ret = USB1_Read_SECTOR_DATA(data, bpb_instance.Starting_LBA, 1, Glob_Sector_data, &Sector_data_len, "Sector starting LBA", 0, 0);
    }
    if (ret == 0){
#if (PRINT_DEBUG)
        USB1_Print_HexVal(Glob_Sector_data + 13, 1, "Sectors per Cluster", LITTLE_ENDIAN);
        USB1_Print_HexVal(Glob_Sector_data + 14, 2, "Reserved Sectors", LITTLE_ENDIAN);
#endif
        bpb_instance.Sectors_per_Cluster = (u8)USB1_Get_Bytes(Glob_Sector_data + 13, 8, LITTLE_ENDIAN);
        bpb_instance.Reserved_Sectors = (u16)USB1_Get_Bytes(Glob_Sector_data + 14, 16, LITTLE_ENDIAN);

#if (PRINT_DEBUG)
        USB1_Print_HexVal(Glob_Sector_data + 16, 2, "Number of FATs", LITTLE_ENDIAN);
        USB1_Print_HexVal(Glob_Sector_data + 36, 4, "Sectors per FAT table", LITTLE_ENDIAN);
#endif
        bpb_instance.Number_of_FATs = (u16)USB1_Get_Bytes(Glob_Sector_data + 16, 16, LITTLE_ENDIAN);
        bpb_instance.Sectors_per_FAT = USB1_Get_Bytes(Glob_Sector_data + 36, 32, LITTLE_ENDIAN);

#if (PRINT_DEBUG)
        USB1_Print_HexVal(Glob_Sector_data + 44, 4, "Root Cluster", LITTLE_ENDIAN);
        USB1_Print_String(Glob_Sector_data + 82, 8, "FS type");
#endif
        bpb_instance.Root_Cluster = USB1_Get_Bytes(Glob_Sector_data + 44, 32, LITTLE_ENDIAN);
        USB1_Get_String(Glob_Sector_data + 82, bpb_instance.FS_type, 8);

        // check FS type is FAT32 or not
        if ((USB1_Compare_String("FAT32", bpb_instance.FS_type, strlen("FAT32")) == 0)){
            bpb_instance.Data_Sector = bpb_instance.Reserved_Sectors + bpb_instance.Number_of_FATs * bpb_instance.Sectors_per_FAT;
#if (PRINT_DEBUG)
            printk("This is FAT32, real data starts at sector: %d = 0x%x\n", bpb_instance.Data_Sector, bpb_instance.Data_Sector);
#endif
        }
        else {
            printk("This not FAT32, not supporting any type other than FAT32\n");
            return -1;
        }

    }

    return ret;
}

int USB1_f_Read_All(struct usb_device_data *data){
    int ret;

    printk("================= [Files and Folders] ==================\n");
    Dir_cluster[Dir_index] = 0x02;
    /* Scan from Cluster 2 */
    ret = USB1_Scan_Cluster(data, (u8*)Next_cluster_data, 0x02, PRINT_DIR, ALL_DIR | ALL_DIR_READ_ALL);

    return ret;
}

/* ================ */
// Function to build the actual directory paths without heap allocation
void build_actual_dirs(u32* paddings, u32 len, u32* out_len) {
    u32 i, j;
    u32 stack_size;
    u32 actual_count;
    u32 idx;
    size_t pre_len;
    size_t remaining;
    size_t slen;
    size_t nlen;
    u8 *p;

    memset(num_direct, 0, sizeof(num_direct));
    memset(all_children_leaves, 0, sizeof(all_children_leaves));
    memset(max_subtree_padding, 0, sizeof(max_subtree_padding));
    memset(parent, -1, sizeof(parent)); // Sets all bytes to 0xFF, which for signed int is -1
    memset(include_flags, 0, sizeof(include_flags));
    memset(chain_indices, 0, sizeof(chain_indices));
    stack_size = 0;
    if (len <= 0) {
        *out_len = 1;
        strcpy(actual_dirs[0], "./");
        return;
    }
    for (i = 0; i < len; i++) {
        while (stack_size > paddings[i]) {
            stack_size--;
        }
        p = path;
        remaining = sizeof(path);
        *p = '\0';
        for (j = 0; j < stack_size; j++) {
            slen = strlen(stack[j]);
            if (slen + 1 >= remaining) {
                slen = remaining - 2;
            }
            memcpy(p, stack[j], slen);
            p += slen;
            *p++ = '/';
            *p = '\0';
            remaining -= (slen + 1);
        }
        nlen = strlen(names[i]);
        if (nlen >= remaining) {
            nlen = remaining - 1;
        }
        memcpy(p, names[i], nlen);
        p += nlen;
        *p = '\0';
        strcpy(full_paths[i], path);
        strcpy(stack[stack_size++], names[i]);
    }
    for (i = 0; i < len; i++) {
        snprintf(prefix, sizeof(prefix), "%s/", full_paths[i]);
        pre_len = strlen(prefix);
        for (j = 0; j < len; j++) {
            if (i != j && strncmp(full_paths[j], prefix, pre_len) == 0 && strchr(full_paths[j] + pre_len, '/') == NULL) {
                num_direct[i]++;
            }
        }
    }
    for (i = 0; i < len; i++) {
        all_children_leaves[i] = true;
        snprintf(prefix, sizeof(prefix), "%s/", full_paths[i]);
        pre_len = strlen(prefix);
        for (j = 0; j < len; j++) {
            if (i != j && strncmp(full_paths[j], prefix, pre_len) == 0 && strchr(full_paths[j] + pre_len, '/') == NULL) {
                if (num_direct[j] != 0) {
                    all_children_leaves[i] = false;
                }
            }
        }
    }
    for (i = 0; i < len; i++) {
        max_subtree_padding[i] = paddings[i];
        snprintf(prefix, sizeof(prefix), "%s/", full_paths[i]);
        pre_len = strlen(prefix);
        for (j = 0; j < len; j++) {
            if (strncmp(full_paths[j], prefix, pre_len) == 0) {
                if (paddings[j] > max_subtree_padding[i]) {
                    max_subtree_padding[i] = paddings[j];
                }
            }
        }
    }
    for (i = 0; i < len; i++) {
        snprintf(prefix, sizeof(prefix), "%s/", full_paths[i]);
        pre_len = strlen(prefix);
        for (j = 0; j < len; j++) {
            if (i != j && strncmp(full_paths[j], prefix, pre_len) == 0 && strchr(full_paths[j] + pre_len, '/') == NULL) {
                parent[j] = i;
            }
        }
    }
    // Set all include_flags to true to include every directory
    for (i = 0; i < len; i++) {
        include_flags[i] = true;
    }
    actual_count = 1;
    for (i = 0; i < len; i++) {
        if (include_flags[i]) {
            actual_count++;
        }
    }
    strcpy(actual_dirs[0], "./");
    idx = 1;
    for (i = 0; i < len; i++) {
        if (include_flags[i]) {
            snprintf(actual_dirs[idx], MAX_PATH_LEN, "./%s", full_paths[i]);
            idx++;
        }
    }
    *out_len = actual_count;
}

int USB1_Scan_Dir_All(struct usb_device_data *data){
    int ret;

    // u8 names[MAX_DIRS][MAX_PATH_LEN]; // up to 10 directories
    memset((u8*)Dir_cluster, 0x00, sizeof(Dir_cluster));
    memset((u8*)Dir_padding, 0x00, sizeof(Dir_padding));
    Dir_index = 0;

    Dir_cluster[Dir_index] = 0x02;

    ret = USB1_Scan_Cluster(data, (u8*)Next_cluster_data, 0x02, NO_PRINT_DIR, ALL_DIR);

    return ret;
}

int USB1_f_Read_Dir(struct usb_device_data *data, u8* path_dir){
    // int i = 0;
    // u8* names[] = {"Countries", "BBB", "New_power", "AAA", "CCC", "VVV", "Languages", "Protocols", "USB", "BULK", "Ethernet"};
    // u32 paddings[] = {0, 1, 1, 1, 0, 1, 0, 0, 1, 2, 1};
    // u32 len = sizeof(names) / sizeof(names[0]);
    // u32 out_len;
    // build_actual_dirs(paddings, len, &out_len);

    // printk("out_len = %d\n", out_len);

    // for (i = 0; i < out_len; i++) {
    //     printk("%s\n", actual_dirs[i]);
    // }
    // return 0;



    // u32 out_len = 0, i = 0, j = 0;
    // bool dir_existed = false;
    // int ret;
    // build_actual_dirs(Dir_padding, Dir_index, &out_len);
    // printk("len = %d\n", out_len);

    // for (i = 0; i < out_len; i++) {
    //     printk("%s\n", actual_dirs[i]);
    // }
    // for (i = 0; i < out_len; i++) {
    //     printk("cluster_num = %d\n", Dir_cluster[i]);
    // }
    // return 0;



    u32 out_len = 0, i = 0, j = 0;
    bool dir_existed = false;
    int ret;

    USB1_Scan_Dir_All(data);
    build_actual_dirs(Dir_padding, Dir_index, &out_len);

    // printk("len = %d\n", out_len);

    // for (i = 0; i < out_len; i++) {
    //     printk("%s\n", actual_dirs[i]);
    // }

    for (i = 0; i < out_len; i++){
        while (actual_dirs[i][j] != '\0'){
            j++;
        }
        j++; // adding '\0'
        
        ret = USB1_Compare_String(path_dir, actual_dirs[i], j);
        j = 0;

        if (ret == 0){
            //printk("YES, index in String = %d\n", i);
            dir_existed = true;
            break;
        }
    }

    if (dir_existed){
        //printk("'%s':\n", path_dir);
        padding_index = 1;
        if (ret == 0) ret = USB1_Scan_Cluster(data, (u8*)Next_cluster_data, Dir_cluster[i], PRINT_DIR, CURRENT_DIR);
        padding_index = 0;

        //USB1_Print_Hex((u8*)Next_cluster_data, SECTOR_SIZE, "Dir");  
    }
    else {
        printk("Invalid dir\n");
        ret = -1;
    }

    return ret;
}

int USB1_f_Read_File(struct usb_device_data *data, u8* path_file){
    u8 File_name[50];
    u8 Dir_name[100];
    u32 File_name_len = 0, Dir_name_len = 0;

    u32 out_len = 0, i = 0, j = 0;
    bool dir_existed = false;
    bool file_existed = false;
    int ret;

    u8* ptr8;
    Root_Dir_Entry* entry;
    u16 next_cluster = 0;
    u32 Cluster_data_len;
    u32 size;


    u16 LFN_ret = 0;
    u8 tmp_file_name[50];
    u16 tmp_file_name_len = 0;

    USB1_Parse_TargetFile(path_file, Dir_name, &Dir_name_len, File_name, &File_name_len);
    // printk("Dir: '%s', File: '%s'\n", Dir_name, File_name);
    // printk("Dir_name_len: %d, File_name_len: %d\n", Dir_name_len, File_name_len);


    USB1_Scan_Dir_All(data);
    build_actual_dirs(Dir_padding, Dir_index, &out_len);
    // printk("len = %d\n", out_len);

    // for (i = 0; i < out_len; i++) {
    //     printk("%s\n", actual_dirs[i]);
    // }

    for (i = 0; i < out_len; i++){
        while (actual_dirs[i][j] != '\0'){
            j++;
        }
        j++; // adding '\0'
        
        ret = USB1_Compare_String(Dir_name, actual_dirs[i], j);
        j = 0;

        if (ret == 0){
            //printk("YES, index in String = %d\n", i);
            dir_existed = true;
            break;
        }
    }

    if (dir_existed){
        if (ret == 0) ret = USB1_Read_CLUSTER(data, Dir_cluster[i], (u8*)Next_cluster_data, &Cluster_data_len, "USB1_f_Read_File", 0);

        ptr8 = (u8*)Next_cluster_data;
        while (ptr8[0]){ // make sure the first char is not zero
            if (ptr8[11] == FILE_TYPE){
                entry = (Root_Dir_Entry*)ptr8; // save here
                //ntry_start = entry;

                // =============== checking the LFN name
                USB1_Read_NameFile(entry, tmp_file_name, &tmp_file_name_len);

                LFN_ret = USB1_Compare_String(File_name, tmp_file_name, tmp_file_name_len);
                // printk("tmp_file_name: '%s', File_name: '%s'\n", tmp_file_name, File_name);
                // printk("RET = %d\n", LFN_ret);
                tmp_file_name_len = 0;
                if (LFN_ret == 0){
                    next_cluster = (entry->High_first_cluster << 16) | (entry->Low_first_cluster);
                    size = entry->File_size;

                    //printk("next_cluster = %d, size = %d\n", next_cluster, size);
                    ret = USB1_Read_CLUSTER(data, next_cluster, (u8*)Cluster_data, &Cluster_data_len, "Reading dir entry", 0);

                    printk("%s\n", path_file);
                    USB1_Print_String((u8*)Cluster_data, size, "->"); 

                    file_existed = true;
                    
                    break;
                }
            }
            ptr8+=32;
        }

        if (file_existed == false) {
            printk("Invalid file\n");
            return -1;
        }
    }

    else {
        printk("Invalid dir\n");
        return -1;
    }

    return 0;
}

/* ======= API for makedir ========= */
u32 USB1_Find_Free_Entry(u8* cluster_data, Root_Dir_Entry** entry){
    u32 free_dir_index = 0;
    (*entry) = (Root_Dir_Entry*)cluster_data;

    //printk("entry = 0x%x - data = 0x%x\n", *entry, cluster_data);

    while ((*entry)->id != 0x00){
        (*entry)++;
        //printk("entry = 0x%x - data = 0x%x\n", *entry, cluster_data);
        free_dir_index++;
    }

    return free_dir_index;
}

bool IsLowercase(u8 c){
    if ((c >= 0x61) && (c <= 0x7A)) return true;
    else if ((c >= 0x41) && (c <= 0x5A)) return false;
    else return false;
}

/* Compute VFAT LFN checksum for an 8.3 short name (11 bytes) */
u8 vfat_lfn_checksum(u8* sfn, u16 len) {
    u8 chk = 0;
    u16 i = 0;

    for (i = 0; i < len; ++i) {
        /* rotate-right by 1, then add next byte */
        chk = ((chk & 1) ? 0x80 : 0) + (chk >> 1) + sfn[i];
    }
    return chk;
}

// dir_name_len not including '\0'
void USB1_Create_Cluster_SFN(u8* dir_name, u32 dir_name_len, u8 type, u32 next_cluster_num, Root_Dir_Entry* entry){
    u32 i = 0;
    bool is;

    u8 tmp[8];

    // printk("dir_name_len = %d\n", dir_name_len);

    // fulfill 0x20
    for (i = 0; i < 8; i++){
        if (i < dir_name_len){
            tmp[i] = dir_name[i];
        }
        else {
            tmp[i] = 0x20;
        }
    }

    entry->id = tmp[0];
    is = IsLowercase(tmp[0]);
    if (is == true) entry->id -= 0x20; // make it become uppercase

    if (type == DIR_TYPE){
        for (i = 1; i < 6; i++){
            entry->File_name[i-1] = tmp[i];

            is = IsLowercase(tmp[i]);
            if (is == true) entry->File_name[i-1] -= 0x20; // make it become uppercase
        }
    }
    else{
        for (i = 1; i < 6; i++){
            entry->File_name[i-1] = tmp[i];

            is = IsLowercase(tmp[i]);
            if (is == true) entry->File_name[i-1] -= 0x20; // make it become uppercase

            if (tmp[i+1] == '.'){
                break;
            }
        }

        while(i < 6){
            entry->File_name[i] = 0x20;
            i++;
        }

    }

    if (dir_name_len > 8) {
        entry->File_name[5] = '~';
        entry->File_name[6] = '1';
    }
    else {
        entry->File_name[5] = tmp[i++];
        is = IsLowercase(tmp[i-1]);
        if (is == true) entry->File_name[5] -= 0x20; // make it become uppercase

        entry->File_name[6] = tmp[i++];
        is = IsLowercase(tmp[i-1]);
        if (is == true) entry->File_name[6] -= 0x20; // make it become uppercase
    }

    if (type == DIR_TYPE){
        // instead of txt in file type
        entry->File_name[7] = 0x20;
        entry->File_name[8] = 0x20;
        entry->File_name[9] = 0x20;
    }
    else {
        entry->File_name[7] = 'T';
        entry->File_name[8] = 'X';
        entry->File_name[9] = 'T';       
    }

    entry->File_attributes = type;

    // next cluster num
    entry->High_first_cluster = (next_cluster_num&0xFFFF0000)>>16;
    entry->Low_first_cluster = (next_cluster_num&0xFFFF);

    // Time
    entry->File_create_time = 0x4C;

    entry->Create_time = 0x7069;
    entry->Create_date = 0x5B6C;
    entry->Access_date = 0x5B6C;

    entry->Modified_time = 0x7069;
    entry->Modified_date = 0x5B6C;
}

// dir_name_len not including '\0'
void USB1_Create_Cluster_LFN(u8* dir_name, u32 dir_name_len, u8 type, LFN_Root_Dir_Entry* entry, u16* entry_num){
    u16 i = 0, j = 0;
    u16 dir_name_index = 0;
    bool is;
    u8 tmp[11];
    
    *entry_num = (dir_name_len + 1)/13;
    if ((dir_name_len + 1)%13) *entry_num+=1;

    memset((u8*)entry, 0x00, 32*(*entry_num));

    //printf("dir_name_len = %d, entry_num = %d\n", dir_name_len, *entry_num);

    // Get SFN for checksum
    //printf("dir_name_len = %d\n", dir_name_len);

    // fulfill 0x20
    if (type == DIR_TYPE){
        for (i = 0; i < 8; i++){
            if (i < dir_name_len){
                tmp[i] = dir_name[i];
            }
            else {
                tmp[i] = 0x20;
            }

            is = IsLowercase(tmp[i]);
            if (is == true) tmp[i] -= 0x20; // make it become uppercase
        }

        if (dir_name_len > 8) {
            tmp[6] = '~';
            tmp[7] = '1';
        }
    }
    else {
        for (i = 0; i < 8; i++){
            if (i < dir_name_len - 4){
                tmp[i] = dir_name[i];
            }
            else {
                tmp[i] = 0x20;
            }

            is = IsLowercase(tmp[i]);
            if (is == true) tmp[i] -= 0x20; // make it become uppercase
        } 

        if (dir_name_len - 4 > 8) {
            tmp[6] = '~';
            tmp[7] = '1';
        }
    }


    if (type == DIR_TYPE){
        // instead of txt in file type
        tmp[8] = 0x20;
        tmp[9] = 0x20;
        tmp[10] = 0x20;
    }
    else {
        tmp[8] = 'T';
        tmp[9] = 'X';
        tmp[10] = 'T';       
    }

#if (PRINT_DEBUG)
    USB1_Print_String(tmp, 11, "SFN");
#endif

    for (i = *entry_num; i > 1 ; i--){
        (entry + i - 1)->id = 0x00&END_MARKER;
        (entry + i - 1)->id |= (*entry_num - i + 1)&SEQ_NUM;

        (entry + i - 1)->File_attributes = LFN_TYPE;
        (entry + i - 1)->checksum = vfat_lfn_checksum(tmp, 11);

        for (j = 0; j < 10; j+=2){
            (entry + i - 1)->File_name1[j] = dir_name[dir_name_index++];
        }

        for (j = 0; j < 12; j+=2){
            (entry + i - 1)->File_name2[j] = dir_name[dir_name_index++];
        }

        for (j = 0; j < 4; j+=2){
            (entry + i - 1)->File_name3[j] = dir_name[dir_name_index++];
        }
    }

    (entry)->id = END_MARKER;
    (entry)->id |= (*entry_num)&SEQ_NUM;
    (entry)->File_attributes = LFN_TYPE;
    (entry)->checksum = vfat_lfn_checksum(tmp, 11);

    //dir_name_len += 1;
    //printf("dir_name_index = %d\n", dir_name_index);

    for (j = 0; j < 10; j+=2){
        if (dir_name_index < dir_name_len + 1) (entry)->File_name1[j] = dir_name[dir_name_index++];
        else {
            (entry)->File_name1[j] = 0xFF;
            (entry)->File_name1[j+1] = 0xFF;
        }
    }

    for (j = 0; j < 12; j+=2){
        if (dir_name_index < dir_name_len + 1) (entry)->File_name2[j] = dir_name[dir_name_index++];
        else {
            (entry)->File_name2[j] = 0xFF;
            (entry)->File_name2[j+1] = 0xFF;
        }
    }

    for (j = 0; j < 4; j+=2){
        if (dir_name_index < dir_name_len + 1) (entry)->File_name3[j] = dir_name[dir_name_index++];
        else {
            (entry)->File_name3[j] = 0xFF;
            (entry)->File_name3[j+1] = 0xFF;
        }
    }

}

/* dir_name_len not including '\0' */
bool USB1_Check_Existing(u8* cluster_data, u8* name, u32 name_len, u8 type){
    char Output[100];
    Root_Dir_Entry* entry;
    u16 OutputLen = 0;
    bool ret;

    name[name_len++] = '\0'; // adding null terminator

    entry = (Root_Dir_Entry*)cluster_data;

    while(entry->id){
        if ((entry->File_attributes == type) && (entry->id != 0xE5)){
            USB1_Read_NameFile(entry, Output, &OutputLen);

            ret = USB1_Compare_String(Output, name, name_len);
            if (ret == 0){
                return true;
            }
        }
        entry++;
    }

    return false;
}

/* dir_name_len not including '\0' */
bool USB1_Check_Deleted(u8* cluster_data, u8* name, u32 name_len, u8 type, Root_Dir_Entry** output_entry){
    char Output[100];
    Root_Dir_Entry* entry;
    u16 OutputLen = 0;
    bool ret;

    name[name_len++] = '\0'; // adding null terminator

    entry = (Root_Dir_Entry*)cluster_data;

    while(entry->id){

        if ((entry->id == 0xE5) && (entry->File_attributes == type)){
            USB1_Read_NameFile((Root_Dir_Entry*)entry, Output, &OutputLen);

            ret = USB1_Compare_String(Output, name, name_len);

            if (ret == 0){
                *output_entry = entry;
                return true;
            }
        }
        
        entry++;
    }

    return false;
}

// dir_name_len not including '\0'
void USB1_Create_Cluster_Dir(u8* cluster_data, u8* dir_name, u32 dir_name_len, u32 next_cluster_num, u16* bytes_occupied){
    u32 free_dir_index = 0;
    u16 Ins_entry_len = 0;
    Root_Dir_Entry* Glob_free_entry;
    bool dir_existed = false;
    bool dir_deleted = false;

    // checking existing dir
    dir_existed = USB1_Check_Existing(cluster_data, dir_name, dir_name_len, DIR_TYPE);

    if (dir_existed == true){
#if (PRINT_DEBUG)
        printk("Dir already existed\n");
#endif
    }
    else {
        // checking deleted dir
        dir_deleted = USB1_Check_Deleted(cluster_data, dir_name, dir_name_len, DIR_TYPE, &Glob_free_entry);
        if (dir_deleted == false) free_dir_index = USB1_Find_Free_Entry(cluster_data, &Glob_free_entry);

        USB1_Create_Cluster_LFN(dir_name, dir_name_len, DIR_TYPE, (LFN_Root_Dir_Entry*)Ins_entry, &Ins_entry_len);
        USB1_Create_Cluster_SFN(dir_name, dir_name_len, DIR_TYPE, next_cluster_num, &Ins_entry[Ins_entry_len]);
        //USB1_Print_Hex((u8*)&Ins_entry[0], 32*(Ins_entry_len + 1) , "LFN + SFN");  
        
        // Glob_free_entry - Ins_entry_len to get the starting LFN
        if (dir_deleted == true) memcpy(Glob_free_entry - Ins_entry_len, (u8*)&Ins_entry[0], 32*(Ins_entry_len + 1));
        else memcpy(Glob_free_entry, (u8*)&Ins_entry[0], 32*(Ins_entry_len + 1));
        *bytes_occupied = 32*(Ins_entry_len + 1);
    }

}

void USB1_Create_Cluster_Dir_Next(u8* cluster_data, u16 next_cluster_num, u16 parent_cluster_num){
    Root_Dir_Entry* entry;
    u16 i = 0;

    // self dir entry
    entry = (Root_Dir_Entry*)cluster_data;
    entry->id = 0x2E;
    for (i = 0; i < 10; i++){
        entry->File_name[i] = 0x20;
    }
    entry->File_attributes = DIR_TYPE;
    entry->High_first_cluster = (next_cluster_num&0xFFFF0000)>>16;
    entry->Low_first_cluster = (next_cluster_num&0xFFFF);

    // Time
    entry->File_create_time = 0x4C;

    entry->Create_time = 0x7069;
    entry->Create_date = 0x5B6C;
    entry->Access_date = 0x5B6C;

    entry->Modified_time = 0x7069;
    entry->Modified_date = 0x5B6C;

    entry++;

    // parent dir entry
    entry->id = 0x2E;
    entry->File_name[0] = 0x2E;
    for (i = 1; i < 10; i++){
        entry->File_name[i] = 0x20;
    }
    entry->File_attributes = DIR_TYPE;
    entry->High_first_cluster = (parent_cluster_num&0xFFFF0000)>>16;
    entry->Low_first_cluster = (parent_cluster_num&0xFFFF);

    // Time
    entry->File_create_time = 0x4C; // 76 × 10 ms = 760 ms (0.76 seconds)

    entry->Create_time = 0x7069; // Hours (bits 15-11): 14 | Minutes (bits 10-5): 3 | Seconds/2 (bits 4-0): 9 → Seconds = 9 × 2 = 18
    entry->Create_date = 0x5B6C; // Year (bits 15-9): 45 → 1980 + 45 = 2025 | Month (bits 8-5): 11 | Day (bits 4-0): 12
    entry->Access_date = 0x5B6C;

    entry->Modified_time = 0x7069;
    entry->Modified_date = 0x5B6C;
}

u16 USB1_Get_Free_Cluster(struct usb_device_data *data){
    int ret = 0;
    u16 Sector_data_len;
    u32* ptr32;
    u16 i = 0;

    ret = USB1_Read_SECTOR_DATA(data, bpb_instance.Starting_LBA + bpb_instance.Reserved_Sectors, 1, Glob_Sector_data, &Sector_data_len, "Get free cluster", 0, 0);
    if (ret == 0){
        ptr32 = (u32*)Glob_Sector_data;

        /* TODO */
        for (i = 0; i < 100; i++){
            if (ptr32[i] == 0x00){
                break;
            }
        }

        return i;
    }

    return 0;
}

void USB1_Read_FAT(struct usb_device_data *data){
    u16 Sector_data_len;

    (void)USB1_Read_SECTOR_DATA(data, bpb_instance.Starting_LBA + bpb_instance.Reserved_Sectors, 1, Glob_Sector_data, &Sector_data_len, "FAT content", 0, NO_PRINT_DATA);
}

void USB1_Set_Used_Cluster(struct usb_device_data *data, u16 cluster_num){
    int ret = 0;
    u16 Sector_data_len;
    u32* ptr32;

    ret = USB1_Read_SECTOR_DATA(data, bpb_instance.Starting_LBA + bpb_instance.Reserved_Sectors, 1, Glob_Sector_data, &Sector_data_len, "Get free cluster", 0, 0);
    if (ret == 0){
        ptr32 = (u32*)Glob_Sector_data;

        /* TODO: assume not chaining cluster here */
        ptr32[cluster_num] = 0x0FFFFFFF;

        ret = USB1_Write_SECTOR_DATA(data, bpb_instance.Starting_LBA + bpb_instance.Reserved_Sectors, 1, Glob_Sector_data, SECTOR_SIZE, "Set free cluster", 0, NO_PRINT_DATA);

        USB1_Read_FAT(data);
    }
}

int USB1_f_Make_Dir(struct usb_device_data *data, u8* path_dir){
    u8 Dir_name1[100];
    u8 Dir_name2[100];
    u32 Dir_name1_len = 0, Dir_name2_len = 0;
    u32 Cluster_data_len;
    u16 bytes_occupied;
    u16 cluster_free_num;

    u32 out_len = 0, i = 0, j = 0;
    bool dir_existed = false;
    int ret;

    USB1_Parse_TargetDir(path_dir, Dir_name1, &Dir_name1_len, Dir_name2, &Dir_name2_len);
    // printk("Dir_name1: '%s', Dir_name2: '%s'\n", Dir_name1, Dir_name2);
    // printk("Dir_name1_len: %d, Dir_name2_len: %d\n", Dir_name1_len, Dir_name2_len);

    // do this first
    USB1_Scan_Dir_All(data);

    build_actual_dirs(Dir_padding, Dir_index, &out_len);

    // printk("out_len = %d\n", out_len);

    // for (i = 0; i < out_len; i++) {
    //     printk("'%s' - %d - %d\n", actual_dirs[i], Dir_cluster[i], Dir_padding[i]);
    // }

    for (i = 0; i < out_len; i++){
        while (actual_dirs[i][j] != '\0'){
            j++;
        }
        j++; // adding '\0'

        //printk("j = %d\n",j);

        if (Dir_name1_len == 1){ // root
            dir_existed = true;

            // get free cluster num from FAT
            cluster_free_num = USB1_Get_Free_Cluster(data);

            // set free cluster num to FAT
            USB1_Set_Used_Cluster(data, cluster_free_num);

            i = 0;
            Dir_cluster[i] = 2;
            Dir_padding[i] = 0;
            ret = 0;

            break;
        }
        else {
            ret = USB1_Compare_String(Dir_name1, actual_dirs[i], j);

            if (ret == 0){
#if (PRINT_DEBUG)
                printk("YES, index in String = %d\n", i);
#endif
                dir_existed = true;

                // get free cluster num from FAT
                cluster_free_num = USB1_Get_Free_Cluster(data);
                //cluster_free_num++;
                // set free cluster num to FAT
                USB1_Set_Used_Cluster(data, cluster_free_num);
#if (PRINT_DEBUG)
                printk("cluster_free_num = %d, Dir_cluster[i] = %d\n", cluster_free_num, Dir_cluster[i]);
#endif
                break;
            }
        }

        j = 0;
        
    }


    if (dir_existed) {
        if (ret == 0) ret = USB1_Read_CLUSTER(data, Dir_cluster[i], (u8*)Next_cluster_data, &Cluster_data_len, "USB1_f_Make_Dir", 0);

        if (ret == 0) {
            // free_entry_index = USB1_Find_Free_Entry((u8*)Next_cluster_data, entry);
            //printk("cluster num = %d, free_entry_index = %d\n", Dir_cluster[i], free_entry_index);

            USB1_Create_Cluster_Dir((u8*)Next_cluster_data, Dir_name2+1, Dir_name2_len - 1, cluster_free_num, &bytes_occupied);
            //USB1_Print_Hex((u8*)Next_cluster_data, SECTOR_SIZE, "Dir");  
            ret = USB1_Write_CLUSTER(data, Dir_cluster[i], (u8*)Next_cluster_data, "Dir", 0);

            memset((u8*)Next_cluster_data, 0x00, 8*SECTOR_SIZE);
            USB1_Create_Cluster_Dir_Next((u8*)Next_cluster_data, cluster_free_num, Dir_cluster[i]);
            //USB1_Print_Hex((u8*)Next_cluster_data, SECTOR_SIZE, "Dir next");  
            ret = USB1_Write_CLUSTER(data, cluster_free_num, (u8*)Next_cluster_data, "Dir next", 0);

            //USB1_Read_FAT(data);

             
        }
    }
    else {
        printk("Invalid dir xxx\n");
        return -1;
    }

    return ret;
}

// ================== rm dir
void USB1_Clear_Used_Cluster(struct usb_device_data *data, u16 cluster_num){
    int ret = 0;
    u16 Sector_data_len;
    u32* ptr32;

    ret = USB1_Read_SECTOR_DATA(data, bpb_instance.Starting_LBA + bpb_instance.Reserved_Sectors, 1, Glob_Sector_data, &Sector_data_len, "Get free cluster", 0, NO_PRINT_DATA);
    if (ret == 0){
        ptr32 = (u32*)Glob_Sector_data;

        /* TODO */
        ptr32[cluster_num] = 0x00;

        ret = USB1_Write_SECTOR_DATA(data, bpb_instance.Starting_LBA + bpb_instance.Reserved_Sectors, 1, Glob_Sector_data, SECTOR_SIZE, "Set free cluster", 0, NO_PRINT_DATA);
    }
}

/* entry_start is a pointer to SFN dir entry */
void USB1_Clear_Cluster_LFN(Root_Dir_Entry* entry_start){
    entry_start--;
    while(!((entry_start->id)&END_MARKER)){
        //memset((u8*)entry_start, 0x00, 32);
        entry_start->id = 0xE5;

        entry_start-= 1; // back the previous entry
    }

    if ((entry_start->id)&END_MARKER){
        //memset((u8*)entry_start, 0x00, 32);
        entry_start->id = 0xE5;
    }
}

// dir_name_len including '\0'
u16 USB1_Clear_Cluster_Dir(struct usb_device_data *data, u8* cluster_data, u8* dir_name, u32 dir_name_len){
    Root_Dir_Entry* SFN_entry;
    u8 tmp_dir[100];
    u16 tmp_dir_len;
    int ret;
    bool folder_existed = false;
    Root_Dir_Entry* Target_SFN_entry;
    u16 starting_cluster;

    SFN_entry = (Root_Dir_Entry*)cluster_data;

    //USB1_Print_Hex(cluster_data, 512, "Clear cluster");

    while(SFN_entry->id){
        if (SFN_entry->File_attributes == DIR_TYPE){
            USB1_Read_NameFile(SFN_entry, tmp_dir, &tmp_dir_len);

            //printk("%s vs %s\n", dir_name, tmp_dir);

            ret = USB1_Compare_String(dir_name, tmp_dir, dir_name_len);
            if (ret == 0){
                folder_existed = true;
                Target_SFN_entry = SFN_entry;

                starting_cluster = (Target_SFN_entry->High_first_cluster<<16) | (Target_SFN_entry->Low_first_cluster);

                // clear cluster num to FAT
                USB1_Clear_Used_Cluster(data, starting_cluster);
#if (PRINT_DEBUG)
                printk("cluster_cleared_num = %d\n", starting_cluster);
#endif
            }
        }

        SFN_entry++;
    }

    //printk("out\n");

    if (folder_existed == true){
        if (Target_SFN_entry->id != 0xE5){
#if (PRINT_DEBUG)
            printk("Yes\n");
#endif
            Target_SFN_entry->id = 0xE5;
            //USB1_Read_FAT(data);
            USB1_Clear_Cluster_LFN(Target_SFN_entry);

            return starting_cluster;
        }
        else {
#if (PRINT_DEBUG)
            printk("Dir already deleted\n");
#endif
        }
    }
    else {
#if (PRINT_DEBUG)
        printk("Dir not existed\n");
#endif
    }

    return 0;
}

void USB1_Clear_Cluster_Dir_Next(u8* cluster_data){
    Root_Dir_Entry* entry;

    // self dir entry
    entry = (Root_Dir_Entry*)cluster_data;
    //USB1_Print_Hex(cluster_data, 512, "Clear cluster next");
    while (entry->id){
        memset((u8*)entry, 0x00, 32);
        entry++;
    }
}

int USB1_f_Remove_Dir(struct usb_device_data *data, u8* path_dir){
    u8 Dir_name1[100];
    u8 Dir_name2[100];
    u32 Dir_name1_len = 0, Dir_name2_len = 0;
    u32 Cluster_data_len;
    u16 cluster_free_num;

    u32 out_len = 0, i = 0, j = 0;
    bool dir_existed = false;
    int ret;

    USB1_Parse_TargetDir(path_dir, Dir_name1, &Dir_name1_len, Dir_name2, &Dir_name2_len);
#if (PRINT_DEBUG)
    printk("Dir_name1: '%s', Dir_name2: '%s'\n", Dir_name1, Dir_name2);
    printk("Dir_name1_len: %d, Dir_name2_len: %d\n", Dir_name1_len, Dir_name2_len);
#endif

    // do this first
    USB1_Scan_Dir_All(data);

    build_actual_dirs(Dir_padding, Dir_index, &out_len);

    // printk("out_len = %d\n", out_len);

    // for (i = 0; i < out_len; i++) {
    //     printk("%s\n", actual_dirs[i]);
    // }

    for (i = 0; i < out_len; i++){
        while (actual_dirs[i][j] != '\0'){
            j++;
        }
        j++; // adding '\0'
#if (PRINT_DEBUG)
        printk("===> %s, num = %d\n", actual_dirs[i], Dir_cluster[i]);
#endif
        
        if (Dir_name1_len == 1){ // root
            i = 0;
            Dir_cluster[i] = 2;
            Dir_padding[i] = 0;
            ret = 0;
            dir_existed = true;

            break;
        }
        else {
            ret = USB1_Compare_String(Dir_name1, actual_dirs[i], j);
            j = 0;

            if (ret == 0){
                printk("YES, index in String = %d\n", i);
                dir_existed = true;
                break;
            }
        }

    }

    // // clear free cluster num to FAT
    // USB1_Clear_Used_Cluster(data, cluster_free_num);
    // printk("cluster_free_num = %d\n", cluster_free_num);

    if (dir_existed) {
        if (ret == 0) ret = USB1_Read_CLUSTER(data, Dir_cluster[i], (u8*)Next_cluster_data, &Cluster_data_len, "USB1_f_Remove_Dir", 0);

        if (ret == 0) {
            // free_entry_index = USB1_Find_Free_Entry((u8*)Next_cluster_data, entry);
#if (PRINT_DEBUG)
            printk("cluster num = %d,\n", Dir_cluster[i]);
#endif

            cluster_free_num = USB1_Clear_Cluster_Dir(data, (u8*)Next_cluster_data, Dir_name2+1, Dir_name2_len - 1);
            //USB1_Print_Hex((u8*)Next_cluster_data, SECTOR_SIZE, "Clr Dir");  
            ret = USB1_Write_CLUSTER(data, Dir_cluster[i], (u8*)Next_cluster_data, "Clr Dir", 0);

            memset((u8*)Next_cluster_data, 0x00, 8*SECTOR_SIZE);
            if (ret == 0) ret = USB1_Read_CLUSTER(data, cluster_free_num, (u8*)Next_cluster_data, &Cluster_data_len, "USB1_f_Remove_Dir 2", 0);
            if (ret == 0){
                USB1_Clear_Cluster_Dir_Next((u8*)Next_cluster_data);
                //USB1_Print_Hex((u8*)Next_cluster_data, SECTOR_SIZE, "Clr Dir next");  
                ret = USB1_Write_CLUSTER(data, cluster_free_num, (u8*)Next_cluster_data, "Clr Dir next", 0);
            }   
        }
    }
    else {
        printk("Invalid dir\n");
        return -1;
    }

    return ret;
}

// =========== Make file && Remove file
// file_name_len not including '\0'
void USB1_Create_Cluster_File(u8* cluster_data, u8* file_name, u32 file_name_len, u32 next_cluster_num, u16* bytes_occupied){
    bool file_existed = false;
    bool file_deleted = false;
    u32 free_dir_index = 0;
    Root_Dir_Entry* Glob_free_entry;
    u16 Ins_entry_len = 0;

    // checking existing file
    file_existed = USB1_Check_Existing(cluster_data, file_name, file_name_len, FILE_TYPE);

    if (file_existed == true){
#if (PRINT_DEBUG)
        printk("File already existed\n");
#endif
    }
    else {
        // checking deleted file
        file_deleted = USB1_Check_Deleted(cluster_data, file_name, file_name_len, FILE_TYPE, &Glob_free_entry);
        if (file_deleted == false) free_dir_index = USB1_Find_Free_Entry(cluster_data, &Glob_free_entry);

        USB1_Create_Cluster_LFN(file_name, file_name_len, FILE_TYPE, (LFN_Root_Dir_Entry*)Ins_entry, &Ins_entry_len);
        USB1_Create_Cluster_SFN(file_name, file_name_len - 4, FILE_TYPE, next_cluster_num, &Ins_entry[Ins_entry_len]); // minus '.txt'
        //USB1_Print_Hex((u8*)&Ins_entry[0], 32*(Ins_entry_len + 1) , "LFN + SFN");  
        
        // Glob_free_entry - Ins_entry_len to get the starting LFN
        if (file_deleted == true) memcpy(Glob_free_entry - Ins_entry_len, (u8*)&Ins_entry[0], 32*(Ins_entry_len + 1));
        else memcpy(Glob_free_entry, (u8*)&Ins_entry[0], 32*(Ins_entry_len + 1));
        *bytes_occupied = 32*(Ins_entry_len + 1);
    }
}

int USB1_f_Make_File(struct usb_device_data *data, u8* path_file){
    u8 File_name[50];
    u8 Dir_name[100];
    u32 File_name_len = 0, Dir_name_len = 0;
    u16 cluster_free_num;
    u16 bytes_occupied;

    u32 out_len = 0, i = 0, j = 0;
    bool dir_existed = false;
    int ret;

    u32 Cluster_data_len;


    USB1_Parse_TargetFile(path_file, Dir_name, &Dir_name_len, File_name, &File_name_len);
    // printk("Dir: '%s', File: '%s'\n", Dir_name, File_name);
    // printk("Dir_name_len: %d, File_name_len: %d\n", Dir_name_len, File_name_len);


    USB1_Scan_Dir_All(data);
    build_actual_dirs(Dir_padding, Dir_index, &out_len);
    // printk("len = %d\n", out_len);

    // for (i = 0; i < out_len; i++) {
    //     printk("'%s' - %d - %d\n", actual_dirs[i], Dir_cluster[i], Dir_padding[i]);
    // }

    for (i = 0; i < out_len; i++){
        while (actual_dirs[i][j] != '\0'){
            j++;
        }
        j++; // adding '\0'
        
        if (Dir_name_len == 1){ // file in root dir
            dir_existed = true;

            i = 0;
            Dir_cluster[i] = 2;
            Dir_padding[i] = 0;
            ret = 0;
            break;
        }
        else {
            ret = USB1_Compare_String(Dir_name, actual_dirs[i], j);
            j = 0;

            if (ret == 0){
                //printk("YES, index in String = %d\n", i);
                dir_existed = true;
                break;
            }
        }
    }

    if (dir_existed){
        if (ret == 0) ret = USB1_Read_CLUSTER(data, Dir_cluster[i], (u8*)Next_cluster_data, &Cluster_data_len, "USB1_f_Make_File", 0);
        if (ret == 0){
            // get free cluster num from FAT
            cluster_free_num = USB1_Get_Free_Cluster(data);
            USB1_Set_Used_Cluster(data, cluster_free_num);

            // USB1_Print_Hex((u8*)Next_cluster_data, 512, path_file);

            USB1_Create_Cluster_File((u8*)Next_cluster_data, File_name, File_name_len, cluster_free_num, &bytes_occupied);

            ret = USB1_Write_CLUSTER(data, Dir_cluster[i], (u8*)Next_cluster_data, "File", 0);
        }
    }

    else {
        printk("Invalid dir\n");
        return -1;
    }

    return 0;
}

// file_name_len including '\0'
u16 USB1_Clear_Cluster_File(struct usb_device_data *data, u8* cluster_data, u8* file_name, u32 file_name_len){
    Root_Dir_Entry* SFN_entry;
    u8 tmp_dir[100];
    u16 tmp_dir_len;
    int ret;
    bool file_existed = false;
    Root_Dir_Entry* Target_SFN_entry;
    u16 starting_cluster;

    SFN_entry = (Root_Dir_Entry*)cluster_data;

    while(SFN_entry->id){
        if (SFN_entry->File_attributes == FILE_TYPE){
            USB1_Read_NameFile(SFN_entry, tmp_dir, &tmp_dir_len);

            ret = USB1_Compare_String(file_name, tmp_dir, file_name_len);
            if (ret == 0){
                file_existed = true;
                Target_SFN_entry = SFN_entry;

                starting_cluster = (Target_SFN_entry->High_first_cluster<<16) | (Target_SFN_entry->Low_first_cluster);

                // clear cluster num to FAT
                USB1_Clear_Used_Cluster(data, starting_cluster);
#if (PRINT_DEBUG)
                printk("File cluster_cleared_num = %d\n", starting_cluster);
#endif
            }
        }

        SFN_entry++;
    }

    //printk("out\n");

    if (file_existed == true){
        if (Target_SFN_entry->id != 0xE5){
            //printk("Yes\n");
            Target_SFN_entry->id = 0xE5;
            USB1_Clear_Cluster_LFN(Target_SFN_entry);

            return starting_cluster;
        }
        else {
#if (PRINT_DEBUG)
            printk("File already deleted\n");
#endif
        }
    }
    else {
#if (PRINT_DEBUG)
        printk("File not existed\n");
#endif
    }

    return 0;
}

int USB1_f_Remove_File(struct usb_device_data *data, u8* path_dir){
    u8 Dir_name[100];
    u8 File_name[100];
    u32 Dir_name_len = 0, File_name_len = 0;
    u32 Cluster_data_len;
    u16 cluster_free_num;

    u32 out_len = 0, i = 0, j = 0;
    bool dir_existed = false;
    int ret;

    USB1_Parse_TargetDir(path_dir, Dir_name, &Dir_name_len, File_name, &File_name_len);
    // printk("Dir_name1: '%s', Dir_name2: '%s'\n", Dir_name, File_name);
    // printk("Dir_name1_len: %d, Dir_name2_len: %d\n", Dir_name_len, File_name_len);

    // do this first
    USB1_Scan_Dir_All(data);
    build_actual_dirs(Dir_padding, Dir_index, &out_len);

    // printk("out_len = %d\n", out_len);

    // for (i = 0; i < out_len; i++) {
    //     printk("%s\n", actual_dirs[i]);
    // }

    for (i = 0; i < out_len; i++){
        while (actual_dirs[i][j] != '\0'){
            j++;
        }
        j++; // adding '\0'
#if (PRINT_DEBUG)
        printk("===> %s, num = %d\n", actual_dirs[i], Dir_cluster[i]);
#endif
        
        if (Dir_name_len == 1){
            i = 0;
            Dir_cluster[i] = 2;
            Dir_padding[i] = 0;
            ret = 0;
            dir_existed = true;

            break;
        }
        else {
            ret = USB1_Compare_String(Dir_name, actual_dirs[i], j);
            j = 0;

            if (ret == 0){
                printk("YES, index in String = %d\n", i);
                dir_existed = true;
                break;
            }
        }
    }

    // // clear free cluster num to FAT
    // USB1_Clear_Used_Cluster(data, cluster_free_num);
    // printk("cluster_free_num = %d\n", cluster_free_num);

    if (dir_existed) {
        if (ret == 0) ret = USB1_Read_CLUSTER(data, Dir_cluster[i], (u8*)Next_cluster_data, &Cluster_data_len, "USB1_f_Remove_File", 0);

        if (ret == 0) {
            cluster_free_num = USB1_Clear_Cluster_File(data, (u8*)Next_cluster_data, File_name + 1, File_name_len);
            ret = USB1_Write_CLUSTER(data, Dir_cluster[i], (u8*)Next_cluster_data, "Clr File", 0);

            memset((u8*)Next_cluster_data, 0x00, 8*SECTOR_SIZE);
            if (ret == 0) ret = USB1_Read_CLUSTER(data, cluster_free_num, (u8*)Next_cluster_data, &Cluster_data_len, "USB1_f_Remove_File 2", 0);
            if (ret == 0){
                memset((u8*)Next_cluster_data, 0x00, 8*SECTOR_SIZE);
                //USB1_Print_Hex((u8*)Next_cluster_data, SECTOR_SIZE, "Clr Dir next");  
                ret = USB1_Write_CLUSTER(data, cluster_free_num, (u8*)Next_cluster_data, "Clr File next", 0);
            }   
        }
    }
    else {
        printk("Invalid dir\n");
        return -1;
    }

    return ret;
}

// =========== Add content && Delete content
void USB1_Add_Cluster_Content(u8* cluster_data, u8* content, u32 content_len){
    Root_Dir_Entry* Glob_free_entry;
    Glob_free_entry = (Root_Dir_Entry*)cluster_data;

#if (PRINT_DEBUG)
    if (Glob_free_entry->id){
        printk("Data will be overwritten\n");
    }
    else {
        printk("data will be added");
    }
#endif

    //Glob_free_entry->File_size = content_len;

    memcpy(cluster_data, content, content_len);
}

int USB1_f_Add_Content(struct usb_device_data *data, u8* path_file, u8* content){
    u8 File_name[50];
    u8 Dir_name[100];
    u8 tmp[50];
    u16 tmp_len;
    u32 File_name_len = 0, Dir_name_len = 0;
    Root_Dir_Entry* SFN_entry;
    Root_Dir_Entry* Target_SFN_entry;
    u32 content_len = 0;

    u32 out_len = 0, i = 0, j = 0;
    bool dir_existed = false;
    bool file_existed = false;
    int ret;

    u32 Cluster_data_len;
    u16 Starting_cluster;


    USB1_Parse_TargetFile(path_file, Dir_name, &Dir_name_len, File_name, &File_name_len);
    // printk("Dir: '%s', File: '%s'\n", Dir_name, File_name);
    // printk("Dir_name_len: %d, File_name_len: %d\n", Dir_name_len, File_name_len);


    USB1_Scan_Dir_All(data);
    build_actual_dirs(Dir_padding, Dir_index, &out_len);
    // printk("len = %d\n", out_len);

    // for (i = 0; i < out_len; i++) {
    //     printk("%s\n", actual_dirs[i]);
    // }

    for (i = 0; i < out_len; i++){
        while (actual_dirs[i][j] != '\0'){
            j++;
        }
        j++; // adding '\0'
        
        if (Dir_name_len == 1){ // file in root dir
            dir_existed = true;

            i = 0;
            Dir_cluster[i] = 2;
            Dir_padding[i] = 0;
            ret = 0;
            break;
        }
        else {
            ret = USB1_Compare_String(Dir_name, actual_dirs[i], j);
            j = 0;

            if (ret == 0){
                //printk("YES, index in String = %d\n", i);
                dir_existed = true;
                break;
            }
        }
    }

    if (dir_existed){
        if (ret == 0) ret = USB1_Read_CLUSTER(data, Dir_cluster[i], (u8*)Next_cluster_data, &Cluster_data_len, "USB1_f_Make_File", 0);
        if (ret == 0){

            SFN_entry = (Root_Dir_Entry*)Next_cluster_data;
            while (SFN_entry->id){
                if ((SFN_entry->id != 0xE5) && (SFN_entry->File_attributes == FILE_TYPE)){
                    USB1_Read_NameFile(SFN_entry, tmp, &tmp_len);

                    ret = USB1_Compare_String(tmp, File_name, tmp_len);
                    if (ret == 0){
                        file_existed = true;
                        Target_SFN_entry = SFN_entry;

                        // get len
                        while(content[content_len] != '\0'){
                            content_len++;
                        }
                        Target_SFN_entry->File_size = content_len;
                        // update content
                        ret = USB1_Write_CLUSTER(data, Dir_cluster[i], (u8*)Next_cluster_data, "File size", 0);
                        break;
                    }
                }
                SFN_entry++;
            }

            if (file_existed == true){

                Starting_cluster = ((Target_SFN_entry->High_first_cluster << 16)&0xFFFF0000) | ((Target_SFN_entry->Low_first_cluster)&0xFFFF);

                if (ret == 0) ret = USB1_Read_CLUSTER(data, Starting_cluster, (u8*)Next_cluster_data, &Cluster_data_len, "USB1_f_Make_File 2", 0);

                if (ret == 0){
                    USB1_Add_Cluster_Content((u8*)Next_cluster_data, content, content_len);
                    // printk("Added!, Starting_cluster = %d\n", Starting_cluster);

                    // USB1_Print_Hex((u8*)Next_cluster_data, 250, "Added");

                    ret = USB1_Write_CLUSTER(data, Starting_cluster, (u8*)Next_cluster_data, "File", 0);

                    // setting FAT
                    USB1_Set_Used_Cluster(data, Starting_cluster);
                }
            }
            else {
                printk("[ERROR] USB1_f_Add_Content -> Invalid filename\n");
                return -1;
            }
        }
    }

    else {
        printk("[ERROR] Invalid dir\n");
        return -1;
    }

    return 0;
}

void USB1_Delete_Cluster_Content(u8* cluster_data){
    memset(cluster_data, 0x00, SECTOR_SIZE*8);
}

int USB1_f_Delete_Content(struct usb_device_data *data, u8* path_file){
    u8 File_name[50];
    u8 Dir_name[100];
    u8 tmp[50];
    u16 tmp_len;
    u32 File_name_len = 0, Dir_name_len = 0;
    Root_Dir_Entry* SFN_entry;
    Root_Dir_Entry* Target_SFN_entry;

    u32 out_len = 0, i = 0, j = 0;
    bool dir_existed = false;
    bool file_existed = false;
    int ret;

    u32 Cluster_data_len;
    u16 Starting_cluster;


    USB1_Parse_TargetFile(path_file, Dir_name, &Dir_name_len, File_name, &File_name_len);
    // printk("Dir: '%s', File: '%s'\n", Dir_name, File_name);
    // printk("Dir_name_len: %d, File_name_len: %d\n", Dir_name_len, File_name_len);


    USB1_Scan_Dir_All(data);
    build_actual_dirs(Dir_padding, Dir_index, &out_len);
    // printk("len = %d\n", out_len);

    // for (i = 0; i < out_len; i++) {
    //     printk("%s\n", actual_dirs[i]);
    // }

    for (i = 0; i < out_len; i++){
        while (actual_dirs[i][j] != '\0'){
            j++;
        }
        j++; // adding '\0'
        
        if (Dir_name_len == 1){ // file in root dir
            dir_existed = true;

            i = 0;
            Dir_cluster[i] = 2;
            Dir_padding[i] = 0;
            ret = 0;
            break;
        }
        else {
            ret = USB1_Compare_String(Dir_name, actual_dirs[i], j);
            j = 0;

            if (ret == 0){
                //printk("YES, index in String = %d\n", i);
                dir_existed = true;
                break;
            }
        }
    }

    if (dir_existed){
        if (ret == 0) ret = USB1_Read_CLUSTER(data, Dir_cluster[i], (u8*)Next_cluster_data, &Cluster_data_len, "USB1_f_Make_File", 0);
        if (ret == 0){

            SFN_entry = (Root_Dir_Entry*)Next_cluster_data;
            while (SFN_entry->id){
                if ((SFN_entry->id != 0xE5) && (SFN_entry->File_attributes == FILE_TYPE)){
                    USB1_Read_NameFile(SFN_entry, tmp, &tmp_len);

                    ret = USB1_Compare_String(tmp, File_name, tmp_len);
                    if (ret == 0){
                        file_existed = true;
                        Target_SFN_entry = SFN_entry;

                        // set len = 0
                        Target_SFN_entry->File_size = 0;
                        // update content
                        ret = USB1_Write_CLUSTER(data, Dir_cluster[i], (u8*)Next_cluster_data, "File size", 0);
                        break;
                    }
                }
                SFN_entry++;
            }

            if (file_existed == true){

                Starting_cluster = ((Target_SFN_entry->High_first_cluster << 16)&0xFFFF0000) | ((Target_SFN_entry->Low_first_cluster)&0xFFFF);

                if (ret == 0) ret = USB1_Read_CLUSTER(data, Starting_cluster, (u8*)Next_cluster_data, &Cluster_data_len, "USB1_f_Make_File 2", 0);

                if (ret == 0){
                    USB1_Delete_Cluster_Content((u8*)Next_cluster_data);
                    // printk("Deleted!, Starting_cluster = %d\n", Starting_cluster);

                    // USB1_Print_Hex((u8*)Next_cluster_data, 250, "Added");

                    ret = USB1_Write_CLUSTER(data, Starting_cluster, (u8*)Next_cluster_data, "File", 0);

                    // setting FAT
                    USB1_Clear_Used_Cluster(data, Starting_cluster);
                }
            }
            else {
                printk("[ERROR] USB1_f_Delete_Content -> Invalid filename\n");
                return -1;
            }
        }
    }

    else {
        printk("[ERROR] USB1_f_Delete_Content -> Invalid dir\n");
        return -1;
    }

    return 0;  
}