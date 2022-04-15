/*
 * mapping.h
 *
 *  Created on: 15 Apr 2022
 *      Author: samul
 */

#ifndef INC_MAPPING_H_
#define INC_MAPPING_H_

#include <stdio.h>

typedef struct Map_holder_{
	unsigned char* map;
    size_t width;
	size_t height;
    size_t mem_size;
} *Map_holder;

Map_holder map_init(size_t width, size_t height);
void map_set(Map_holder map_holder, size_t x, size_t y, unsigned char value);
void write_map_file(Map_holder map_holder, char* filename);
void map_delete(Map_holder map_holder);

#endif /* INC_MAPPING_H_ */
