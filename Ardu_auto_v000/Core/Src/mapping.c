/*
 * mapping.c
 *
 *  Created on: 15 Apr 2022
 *      Author: samul
 */
#include "mapping.h"
#include "main.h"
#include <L3G4200D.h>
#include <string.h>
#include <stdlib.h>
#include "bt_control.h"
#include "motor_control.h"

Map_holder map_init(size_t width, size_t height){
    size_t memory_size = width * height * sizeof(unsigned char);
    Map_holder map_holder = malloc(memory_size + 3 * sizeof(size_t));
    map_holder->map = malloc(memory_size);
    map_holder->width = width;
    map_holder->height = height;
    map_holder->mem_size = memory_size;
    return map_holder;
}

void map_set(Map_holder map_holder, size_t x, size_t y, unsigned char value){
    if(x < map_holder->width && y < map_holder->height){
        map_holder->map[y * map_holder->width + x] = value;
    }
    else
        printf("map_set out of bounds\n");
}

void write_map_file(Map_holder map_holder, char* filename){
    //FILE *file_ptr;
    /* TODO
    write header:
    "P1\n{width} {height}\n{map string here}"
    */
}

void map_delete(Map_holder map_holder) {
    if (map_holder != NULL) {
        free(map_holder->map);
        free(map_holder);
    }
}

