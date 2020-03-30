#pragma once
/*
 * mmap.h
 *
 *  Created on: Jan 1, 2020
 *      Author: Richard Ensor
 *      Team: Richard Ensor | Shabaz khan | Faber Alarcon | Zachary Kemplar
 */

 // HEADER FILE PROTECTION
#ifndef MMAP_H_
#define MMAP_H_

// LIBRARIES
#include <stdio.h> // Standard I/O
#include <stdlib.h> // Standard library
#include <ctype.h>
#include <string.h> // String type
#include <errno.h> // Standard Errors
#include <stdbool.h> // Boolean conditionals

//DEFINE
#define BLOCKS 0 // num blocks in grid
#define XVALUE 16 // x axis of grid
#define YVALUE 16  // y axis of grid
#define TARGETX1 8 // Center 4 blocks of grid
#define TARGETY1 8 // "
#define TARGETX2 9 // "
#define TARGETY2 9 // "
#define max_line 40
#define max_input 40

// Direction of robot at grid location 
typedef enum orientation { RIGHT, LEFT, UP, DOWN } nose;

// Struct holding box status and position
typedef struct POSITION {
    int x_location; // Node x location in grid
    int y_location; // Node y location in grid
    bool traversed; // has the node been traversed 
    bool detect_D; // wall found?
    bool detect_U; // "
    bool detect_L; // "
    bool detect_R; // " 
    struct POSITION* UP; // Pointer to node above current node
    struct POSITION* DOWN; // Pointer to node below current node
    struct POSITION* LEFT; // Pointer to node to the left of current node
    struct POSITION* RIGHT; // Pointer to node to the right of current node
} NODE, * NODE_PTR;

// Struct holding box status and position
typedef struct MOUSE {
    enum orientation front; // Holds orientation of mm
    int x_location; // Node x location in grid
    int y_location; // Node y location in grid
    struct MOUSE* UP; // Pointer to node above current node
    struct MOUSE* DOWN; // Pointer to node below current node
    struct MOUSE* LEFT; // Pointer to node to the left of current node
    struct MOUSE* RIGHT; // Pointer to node to the right of current node
} MICRO, * MICRO_PTR;

//CONSTANTS 

// GLOBAL VARIABLES 
nose currDirection = UP;

// FUNCTIONAL PROTOTYPES(DECLARATION)
void dataStruct();
void setDimensions(NODE_PTR* hNode, NODE_PTR* tNode, int num_blocks);
void create_grid(NODE_PTR** headNode, NODE_PTR** tailNode);
NODE_PTR setNode(VOID);
MICRO_PTR initMouse(VOID);
void tagMouse(MICRO_PTR* head, MICRO_PTR* tail, MICRO_PTR* loc, MICRO_PTR newNode);
void tagNode(NODE_PTR*** head, NODE_PTR*** tail, NODE_PTR newNode);
void deleteNode();
void searchAlgo(MICRO_PTR* location, MICRO_PTR headNode);
MICRO_PTR reverseMouse(MICRO_PTR desiredNode, MICRO_PTR tail, MICRO_PTR *currLoc);
MICRO_PTR anotherPath(MICRO_PTR loc);
int manhattanDistance(MICRO_PTR location);
int sourceDistance(MICRO_PTR headNode, MICRO_PTR target);
void moveUp();
void moveLeft();
void moveRight();
void moveDown();
void printMap(NODE_PTR head);
void Exit(NODE_PTR* head);


#endif 
