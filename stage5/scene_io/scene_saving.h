#ifndef scene_saving_h
#define scene_saving_h
#include "../../stage1/master_header.h"
#include "../../stage2/master_header_2.h"
//Saves the current state of objects to a file to be read and loaded
//1 = Successful save, 0 = Failed Save
int save_scene (const char *filedirec);
#endif
