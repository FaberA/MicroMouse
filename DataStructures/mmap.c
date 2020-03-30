/*
 * mmap.c
 *
 *  Created on: Jan 1, 2020
 *      Author: Richard Ensor
 *      Team: Richard Ensor | Shabaz khan | Faber Alarcon | Zachary Kemplar
 *
 *	 University of Maryland, Baltimore County (UMBC)
 *
 *		   IEEE Micromouse Team - XII
 *
 */

 /* MAZE SPECIFICTIONS
  *
  * ---- Dimentions ----
  * Grid: 16x16 blocks
  * Block Size: 18cmx18cm
  * Diameter of wheel: 23mm
  */

//INCLUDED HEADER FILES
#include "mmap.h"

// INCREMENTAL TESTING 

/**************       TEST 0      ***************/
/* The following test is used to examine if         */
/*  */
/*  */
/* uncomment the following line when running test 0 */
//#define TEST_PART_0

/**************       TEST 1      ***************/
/* The following test is used to examine if the     */
/*  */
/*  */
/* uncomment the following line when running test 1 */
//#define TEST_PART_1

/**************       TEST 2      ***************/
/* The following test is used to examine if         */
/*  */
/*  */
/* uncomment the following line when running test 2 */
//#define TEST_PART_2

/**************       TEST 3      ***************/
/* The following test is used to examine if         */
/*  */
/*  */
/* uncomment the following line when running test 3 */
//#define TEST_PART_3

/**************       TEST 4      ***************/
/* The following test is used to examine if         */
/*  */
/*  */
/* uncomment the following line when running test 4 */
//#define TEST_PART_4

/**************       TEST 5      ***************/
/* The following test is used to examine if         */
/*  */
/*  */
/* uncomment the following line when running test 5 */
//#define TEST_PART_5


// ---------------------------------- MAIN ----------------------------------- //
void dataStruct()
{
    // default grid dimensions
    int blocks = BLOCKS;

    // Create Initial Nodes
    NODE_PTR head = NULL;
    NODE_PTR tail = NULL;
    MICRO_PTR mouseHead = NULL; // Micromouse Head
    MICRO_PTR mouseTail = NULL; // Micromouse Tail
    MICRO_PTR location = NULL; // Micromouse current location


    //Function calls
    setDimensions(&head, &tail, blocks);
    mouseHead = initMouse();
    location = mouseHead; // set initial location
    tagMouse(&mouseHead, &mouseTail, &location, mouseHead);
    searchAlgo(&location, mouseHead); 
    Exit(&head);
    printMap(head);
}

//FUNCTION INITIALIZATION 

/* -------------------------------------------------------------------------------
 * FUNCTION NAME : setDimenstions
 * PARAMETERS    : NODE_PTR* hNode, NODE_PTR* tNode, int num_blocks
 * DESCTRIPTION  : Determine grid size and initialize block
 * -------------------------------------------------------------------------------
*/
void setDimensions(NODE_PTR* hNode, NODE_PTR* tNode, int num_blocks)
{
    num_blocks = XVALUE * YVALUE;

    for (int z = 0; z < num_blocks; z++) {
        create_grid(&hNode, &tNode);
    }
}
/* -------------------------------------------------------------------------------
 * FUNCTION NAME : create_grid
 * PARAMETERS    : NODE_PTR *headNode, NODE_PTR *tailNode
 * DESCTRIPTION  : Initializes grid with nodes based on hard coded constraints. 
 * -------------------------------------------------------------------------------
*/
void create_grid(NODE_PTR** headNode, NODE_PTR** tailNode)
{
    // initialize nodes
    NODE_PTR newNode = NULL; //Grid

    /*Grid init*/
    newNode = setNode();
    tagNode(&headNode, &tailNode, newNode);
}

/* ------------------------------------------------------------------------------
 * FUNCTION NAME : setNode
 * PARAMETERS    : VOID
 * DESCTRIPTION  : Creates new Node to be inserted into quad Linked-List
 * -------------------------------------------------------------------------------
*/
NODE_PTR setNode(VOID)
{
    // Allocate memory block for node (malloc is uninitialized)
    NODE_PTR newNode;
    newNode = (NODE_PTR)malloc(sizeof(NODE));

    // Initialize node contents and pointers
    //-----------------------------------------------
    newNode->x_location = 0;
    newNode->y_location = 0;
    newNode->traversed = true;

    newNode->RIGHT = NULL;
    newNode->LEFT = NULL;
    newNode->UP = NULL;
    newNode->DOWN = NULL;

    return newNode;
}

/* ------------------------------------------------------------------------------
 * FUNCTION NAME : initMouse
 * PARAMETERS    : VOID
 * DESCTRIPTION  : Creates new Node to be inserted into mouse Linked-List
 * -------------------------------------------------------------------------------
*/
MICRO_PTR initMouse(VOID)
{
    // Allocate memory block for node (malloc is uninitialized)
    MICRO_PTR mouse = NULL, loc = NULL;
    mouse = (MICRO_PTR)malloc(sizeof(MICRO));

    // Initialize node contents and pointers
    //-----------------------------------------------
    mouse->x_location = 0;
    mouse->y_location = 0;
    mouse->front = UP;

    mouse->RIGHT = NULL;
    mouse->LEFT = NULL;
    mouse->UP = NULL;
    mouse->DOWN = NULL;

    return mouse; 
}

/* ------------------------------------------------------------------------------
 * FUNCTION NAME : tagMouse
 * PARAMETERS    : MICRO_PTR *head, MICRO_PTR *tail, MICRO_PTR newNode
 * DESCTRIPTION  : inserts node into mouse Linked-List
 * -------------------------------------------------------------------------------
*/
void tagMouse(MICRO_PTR *head, MICRO_PTR *tail, MICRO_PTR *loc, MICRO_PTR newNode)
{
    // Set current node to the head of the list
    MICRO_PTR currNode = *head;

    // If there is no data in head create head with newData
    if (*head == NULL)
    {
        *head = newNode;
        newNode->x_location = 0;
        newNode->y_location = 0;
        newNode->front = currDirection; 
    }

    // Determine the direction of the move and tage node into new block 
    if (newNode->front == UP)
    {
        currNode->UP = newNode;
        newNode->UP = currNode;
        newNode->UP = NULL;
        newNode->y_location = currNode->y_location + 1;
        newNode->x_location = currNode->x_location;
    }
    else if (newNode->front == DOWN)
    {
        currNode->DOWN = newNode;
        newNode->UP = currNode;
        newNode->DOWN = NULL;
        newNode->y_location = currNode->y_location - 1;
        newNode->x_location = currNode->x_location;
    }
    else if (newNode->front == LEFT)
    {
        currNode->LEFT = newNode; // initialize empty node with newData
        newNode->RIGHT = currNode;
        newNode->LEFT = NULL;
        newNode->x_location = currNode->x_location - 1;
        newNode->y_location = currNode->y_location;
    }
    else if (newNode->front == RIGHT)
    {
        currNode->RIGHT = newNode; // initialize empty node with newData
        newNode->LEFT = currNode;
        newNode->RIGHT = NULL;
        newNode->x_location = currNode->x_location + 1;
        newNode->y_location = currNode->y_location;
    }

    *tail = newNode; // set tail for reverse 
    loc = tail; 
}

/* ------------------------------------------------------------------------------
 * FUNCTION NAME : reverseMouse
 * PARAMETERS    : MICRO_PTR desiredNode, MICRO_PTR tail, MICRO_PTR *currLoc
 * DESCTRIPTION  : reverses mouse back on same path
 * -------------------------------------------------------------------------------
*/
MICRO_PTR reverseMouse(MICRO_PTR desiredNode, MICRO_PTR tail, MICRO_PTR *currLoc)
{
    // Preserve integrity of tail
    MICRO_PTR currNode = tail;
    MICRO_PTR currLocation = tail;

    while (currNode != desiredNode)
    {
        // Determine the direction of the move and tage node into new block 
        if (currNode->LEFT != NULL)
        {
            currNode = currNode->LEFT;
            moveLeft(); 
        }
        else if (currNode->RIGHT != NULL)
        {
            currNode = currNode->RIGHT;
            moveRight();
        }
        else if (currNode->DOWN != NULL)
        {
            currNode = currNode->DOWN;
            moveDown();
        }
        else if (currNode->UP != NULL)
        {
            currNode = currNode->UP;
            moveUp();
        }
    }
    return currLocation; // return current location of MM
}

/* ------------------------------------------------------------------------------
 * FUNCTION NAME : anotherPath
 * PARAMETERS    : MICRO_PTR loc
 * DESCTRIPTION  : reverses mouse back on same path to last Vertex split tries 
 that path to determine if weight is lower than the sum of the path and its 
 next decision. 
 * -------------------------------------------------------------------------------
*/
MICRO_PTR anotherPath(MICRO_PTR loc)
{
    MICRO_PTR pathNode = NULL; 
    pathNode = loc; 

    return pathNode; 
}

/* -------------------------------------------------------------------------------
 * FUNCTION NAME : tagNode
 * PARAMETERS    : NODE_PTR **head, NODE_PTR **tail,  NODE_PTR newNode
 * DESCTRIPTION  : If head creates a new starting node for quad linked-list
  else Inserts a node to the end of a linked list (tail).
 * -------------------------------------------------------------------------------
*/
void tagNode(NODE_PTR*** head, NODE_PTR*** tail, NODE_PTR newNode)
{
    // Set current node to the head of the list
    NODE_PTR currNode = ***head;

    //Parser Node
    NODE_PTR tempNode = ***head;

    // If there is no data in head create head with newData
    if (***head == NULL)
    {
        ***head = newNode;
        newNode->x_location = 0;
        newNode->y_location = 0;
        newNode->detect_D = true; // Start always has wall behind it 
    }
    else if (XVALUE > currNode->x_location) // else check contents of the next node
    {
        while (currNode->RIGHT != NULL)
        {
            currNode = currNode->RIGHT; // shift to next node
        }

        currNode->RIGHT = newNode; // initialize empty node with newData
        newNode->LEFT = currNode;
        newNode->RIGHT = NULL;
        newNode->x_location = currNode->x_location + 1;
        newNode->y_location = currNode->y_location;

    }
    else if (YVALUE > currNode->y_location)
    {
        while (currNode->DOWN != NULL)
        {
            currNode = currNode->DOWN;
        }

        if (currNode->x_location != 0)
        {
            tempNode = currNode->UP; //hold top node for zipping 
            tempNode = tempNode->LEFT;

            while (currNode->LEFT != NULL)
            {
                currNode = currNode->LEFT;
                tempNode = tempNode->LEFT;
            }

            currNode->LEFT = newNode;
            tempNode->DOWN = newNode;
            newNode->RIGHT = currNode;
            newNode->UP = tempNode;
            newNode->LEFT = NULL;
            newNode->x_location = currNode->x_location - 1;
            newNode->y_location = currNode->y_location;

        }
        else
        {
            while (currNode->RIGHT != NULL)
            {
                currNode = currNode->RIGHT;
            }

            currNode->DOWN = newNode;
            newNode->UP = currNode;
            newNode->y_location = currNode->y_location + 1;
            newNode->x_location = currNode->x_location;
        }
    }
    ***tail = newNode;
}

/* -------------------------------------------------------------------------------
 * FUNCTION NAME : searchAlgo
 * PARAMETERS    :
 * DESCTRIPTION  : Form of breadth-first A* search algorithm specifically an
 implementation of Dijkstras algorithm with Manhattan Distance Heuristic.
 
 Sudo implementation: 

                    INITIALIZE-SINGLE-SOURCE (G, w, s)
                       S = 0
                       Q = G.V
                            while Q = 0
                                m = EXTRACT-MIN(Q)
                                S = S u {m}
                                for each vertex V E G.adj[u]
                                    Relax(u,v,w)

 KEY: 
- G -> weighted directed graph G = (V,E) for the case where E c All POS(+) R
- V -> set of all vertacies in G
- w -> edges such that w(u,v) >= 0 and w c E
- d -> distance from vertacy V to source s 
- S -> set of all vertacies V whos shortest path from the source s have already
  been determined
- u -> minimum shortest path such that u c V-S
- Q -> min heap Data struct for subset of all traversed nodes on shortest 
  path estimate
 * -------------------------------------------------------------------------------
*/
void searchAlgo(MICRO_PTR *location, MICRO_PTR headNode)
{
    MICRO_PTR currNode = *location; 
    MICRO_PTR head = headNode; 

    int numTargets = 0; // how many nodes are not blocked
    int weight = 0; // Sum of the heuristic and determined componants w = (h + d) 
    int heuristic = 0; // heuristic value calculated using optimized mathmatical algo
    int distance = 0; // movement cost to move from starting point to point on grid

    //determine if there is wall in given direction

    heuristic = manhattanDistance(currNode); 
    distance = sourceDistance(head, currNode); 

}

/* -------------------------------------------------------------------------------
 * FUNCTION NAME : manhattanDistance
 * PARAMETERS    :
 * DESCTRIPTION  : MM re-orient to move in desired direction (best for 4 dir grid)
 if design changes to 8 dir grid euclids method may be more optimal. 
 * -------------------------------------------------------------------------------
*/
int manhattanDistance(MICRO_PTR location)
{
    int h1 = 0, h2 = 0, h3 = 0, h4 = 0, minHeuristic;
    MICRO_PTR currNode = NULL;
    currNode = location; 

    minHeuristic = h1; // set default min 

    h1 = abs(currNode->x_location - TARGETX1) + abs(currNode->y_location - TARGETY1); 
    h2 = abs(currNode->x_location - TARGETX2) + abs(currNode->y_location - TARGETY1);
    h3 = abs(currNode->x_location - TARGETX1) + abs(currNode->y_location - TARGETY2);
    h4 = abs(currNode->x_location - TARGETX2) + abs(currNode->y_location - TARGETY2); 

    if (h2 < minHeuristic)
    {
        minHeuristic = h2;
    }
    else if (h3 < minHeuristic)
    {
        minHeuristic = h3;
    }
    else if (h4 < minHeuristic)
    {
        minHeuristic = h4;
    }

    return minHeuristic; 
}

/* -------------------------------------------------------------------------------
 * FUNCTION NAME : sourceDistance
 * PARAMETERS    :
 * DESCTRIPTION  : Cost to move from starting point ot given square
 * -------------------------------------------------------------------------------
*/
int sourceDistance(MICRO_PTR headNode, MICRO_PTR target)
{
    int distance = 0;
    MICRO_PTR head = NULL, targetNode = NULL;
    head = headNode; 
    targetNode = target; 

    if (currDirection == UP)
    {
        distance = abs(targetNode->x_location - head->x_location) + abs(targetNode->y_location + 1 - head->x_location);
    }
    else if (currDirection == DOWN)
    {
        distance = abs(targetNode->x_location - head->x_location) + abs(targetNode->y_location - 1 - head->x_location);
    }
    else if (currDirection == LEFT)
    {
        distance = abs(targetNode->x_location - 1 - head->x_location) + abs(targetNode->y_location - head->x_location);
    }
    else if (currDirection == RIGHT)
    {
        distance = abs(targetNode->x_location + 1 - head->x_location) + abs(targetNode->y_location - head->x_location);
    }

    return distance;
}

/* -------------------------------------------------------------------------------
 * FUNCTION NAME : movUP
 * PARAMETERS    :
 * DESCTRIPTION  : MM re-orient to move in desired direction
 * -------------------------------------------------------------------------------
*/
void moveUp()
{
    /* implement corresponding movement */
}

/* -------------------------------------------------------------------------------
 * FUNCTION NAME : movDown
 * PARAMETERS    :
 * DESCTRIPTION  : MM re-orient to move in desired direction
 * -------------------------------------------------------------------------------
*/
void moveDown()
{
    /* implement corresponding movement */
}

/* -------------------------------------------------------------------------------
 * FUNCTION NAME : movLeft
 * PARAMETERS    :
 * DESCTRIPTION  : MM re-orient to move in desired direction
 * -------------------------------------------------------------------------------
*/
void moveLeft()
{
    /* implement corresponding movement */
}

/* -------------------------------------------------------------------------------
 * FUNCTION NAME : movRight
 * PARAMETERS    :
 * DESCTRIPTION  : MM re-orient to move in desired direction
 * -------------------------------------------------------------------------------
*/
void moveRight()
{
    /* implement corresponding movement */
}

void deleteNode()
{
    /** implemented upon exit at the moment **/
}

/* -------------------------------------------------------------------------------
 * FUNCTION NAME : printMap
 * PARAMETERS    : NODE_PTR head
 * DESCTRIPTION  : Prints the map
 * -------------------------------------------------------------------------------
*/
void printMap(NODE_PTR head)
{
    NODE_PTR MM = head;
    /* Gui printing done in MicroMouse.cpp this implements teensy txt print */
}

/* -------------------------------------------------------------------------------
 * FUNCTION NAME : Exit
 * PARAMETERS    : NODE_PTR *head
 * DESCTRIPTION  : Exits the program
 * -------------------------------------------------------------------------------
*/
void Exit(NODE_PTR* head)
{
    //Free memory
    NODE_PTR currNode = NULL, downNode = NULL, nextNode = NULL, nextDown = NULL;
    currNode = *head;
    downNode = *head;
    int i = 0, v = 0;

    while (downNode != NULL)
    {
        nextDown = downNode->DOWN;

        while (currNode != NULL)
        {
            nextNode = currNode->RIGHT;
            printf("IN: %d \n", i);
            i++;
            free(currNode);
            currNode = nextNode;
        }

        free(currNode);
        printf("OUT: %d \n", v);
        v++;
        currNode = nextDown;
        downNode = nextDown;
    }
    free(nextDown);
    //exit
    exit(0);
}

// TESTING 
#ifdef TEST_PART_0

#endif
#ifdef TEST_PART_1

#endif
#ifdef TEST_PART_2

#endif
#ifdef TEST_PART_3

#endif
#ifdef TEST_PART_4

#endif
#ifdef TEST_PART_5

#endif
