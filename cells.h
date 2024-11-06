#ifndef CELLS_H_
#define CELLS_H_
#include "stdint.h"
#include <limits.h>
#define MAZE_SIZE 16
typedef struct {
  uint32_t cell_val ;
  uint16_t row_position ;
  uint16_t col_position ;
  uint8_t front_wall;
  uint8_t right_wall;
  uint8_t left_wall;
  uint8_t back_wall;
} cell_t ;
typedef struct {
    int x;
    int y;
} point_t;
//cell_t maze [MAZE_SIZE][MAZE_SIZE];
typedef enum {
  no_wall,
  there_wall
}wall_status;
typedef enum {
	head_to_up,
	head_to_right,
	head_to_left,
	head_to_down,
  last_head_start
}robot_position_t;


void set_cell_walls(cell_t maze [MAZE_SIZE][MAZE_SIZE], cell_t * cell);
//void check_access_cells(cell_t maze [MAZE_SIZE][MAZE_SIZE] ,cell_t * current_cell);
void update_maze_val (cell_t maze [MAZE_SIZE][MAZE_SIZE]);
void goto_smallest_val(cell_t maze [MAZE_SIZE][MAZE_SIZE], cell_t * cell);
void init_all_cells(cell_t maze [MAZE_SIZE][MAZE_SIZE] , uint16_t size );
void print_position(cell_t maze [MAZE_SIZE][MAZE_SIZE] , uint16_t size);
void move_cell(cell_t * current_cell , cell_t * min_neighbor_cell);
#endif
