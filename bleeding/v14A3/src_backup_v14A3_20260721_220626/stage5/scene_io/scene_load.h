#ifndef scene_load_h
#define scene_load_h
#include "../../stage1/master_header.h"
#include "../../stage2/master_header_2.h"
//Loads the objects in question from the source file, and repalces the current loaded scene
//1 = Successful load, 0 = Failed load
int scene_loading (const char *file_source_path);
#endif
