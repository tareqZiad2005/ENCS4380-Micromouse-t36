

#include "mouse_hal.h"
#include <stdio.h>
#include <string.h>

#define MAX_SIZE  MM_MAX_SIZE
#define MAX_CELLS (MAX_SIZE * MAX_SIZE)



/*
In our implementation the robot contains, checks and updates some well known attributes and variables so he can proceed
per each cell there are 3 arrays, walls, knwn, and visited
walls is an array where each member is 8 bits, we only use 4, each bit corresponds to one direction, north 0, east bit 1, south bit 2 and bit 3 is west
when the robot find a wall in a cell it updates the corresponding cells (main cell and neighbor) walls status.

same concept of known, but it plays a different role, if we found that a cell has a wall from it's neighbor, we knows that this cell has wall in direction x but for the other edirections we don't, so it's like a checklist of each direction for each cell (if we know there is/not a wall)

visited is used for the exploration to know if we visited all cells
*/



static mm_maze_t maze;
static uint8_t walls[MAX_SIZE][MAX_SIZE];
static uint8_t known[MAX_SIZE][MAX_SIZE];
static uint8_t visited[MAX_SIZE][MAX_SIZE];


/*
the robot keeps it's current cordinates*/

static int mouse_x = 0;
static int mouse_y = 0;
static int heading = MM_NORTH; //initially it starts in north


//those 2 variables would calc for us what would be the new coordinated if we went to a specefic direction
static const int dx[4] = { 0, 1, 0, -1 };
static const int dy[4] = { 1, 0, -1, 0 };

static int in_bounds(int x, int y)      //if a cell is out of the maze bounds or not 
{
    return x >= 0 && y >= 0 && x < maze.width && y < maze.height;
}

static int is_goal_cell(int x, int y)
{
    return x >= maze.goal_x0 && x <= maze.goal_x1 && y >= maze.goal_y0 && y <= maze.goal_y1;
}



/*
this function checks wether a particular sde of a cell is a wall or open path*/
static void record_side(int x, int y, int dir, int is_wall)
{
    int nx, ny;
    int opposite;

    if (!in_bounds(x, y)) return;

    dir = (dir + 4) % 4;
    opposite = (dir + 2) % 4;   //we calculated the oposite direction cause if for exammpe a cell has a wall in it's north then it's nebhor would have a wall in it's south 
    nx = x + dx[dir];
    ny = y + dy[dir]; 

    if (!in_bounds(nx, ny)) is_wall = 1;

    known[y][x] |= MM_WALL(dir);
    if (is_wall) walls[y][x] |= MM_WALL(dir);
    else    walls[y][x] &= (uint8_t)~MM_WALL(dir);

    //if a wall set the corespoding bit to 1 if not set to 0 and mark as known 
    if (in_bounds(nx, ny)) {
        known[ny][nx] |= MM_WALL(opposite);
        if (is_wall) walls[ny][nx] |= MM_WALL(opposite);
        else       walls[ny][nx] &= (uint8_t)~MM_WALL(opposite);
    }
}

static void initialize_boundaries(void)
{
    int x, y;
    for (y = 0; y < maze.height; ++y) {
        for (x = 0; x < maze.width; ++x) {
            if (y == maze.height - 1) record_side(x, y, MM_NORTH, 1);
            if (x == maze.width - 1)  record_side(x, y, MM_EAST, 1);
            if (y == 0)               record_side(x, y, MM_SOUTH, 1);
            if (x == 0)               record_side(x, y, MM_WEST, 1);
        }
    }
}


static void record_three_sensor_directions(uint8_t sensor)
/*
mm_sense gives us the senor's reading but it's realtive to it's heading so we ned to convert that into north west east south*/
{
    int left_dir = (heading + 3) % 4;
    int front_dir = heading;
    int right_dir = (heading + 1) % 4;
        //we check the readings of the sensor and give it to the record side funciton to mark walls and knowns
    record_side(mouse_x, mouse_y, left_dir,(sensor & MM_SEE_LEFT) != 0);
    record_side(mouse_x, mouse_y, front_dir,(sensor & MM_SEE_FRONT) != 0);
    record_side(mouse_x, mouse_y, right_dir,(sensor & MM_SEE_RIGHT) != 0);
}

/*this function reads the walls around a cell and give them as input for the previous function which in returns call teh record sides function that marks the walls and knowns*/
static void observe_current_cell(void)
{
    uint8_t sensor = mm_sense();    //sensor reading 
    record_three_sensor_directions(sensor);
    mm_mark(mouse_x, mouse_y, MM_COLOR_BLUE);
}


static int face_direction(int target_dir)
{

    //this function makes the rbot turns into a spe ific direciton 

    int turns;
    target_dir = (target_dir + 4) % 4;
    turns = (target_dir - heading + 4) % 4;

    if (turns == 0) return 1;
    if (turns == 3) turns = -1;

    if (mm_turn(turns) != MM_OK) {
        mm_log("ERROR: turn failed at (%d,%d)", mouse_x, mouse_y);
        return 0;
    }

    heading = target_dir;
    return 1;
}

static int move_one_cell(int dir)
{
    //this function moves the rbot one cell in a spedcefic direction and only updates the robot coordinates if the moving was successfull
    int nx, ny;
    mm_status_t status;

    dir = (dir + 4) % 4;
    nx = mouse_x + dx[dir];
    ny = mouse_y + dy[dir];

    if (!in_bounds(nx, ny)) {
        record_side(mouse_x, mouse_y, dir, 1);
        return 0;
    }

    if ((known[mouse_y][mouse_x] & MM_WALL(dir)) && (walls[mouse_y][mouse_x] & MM_WALL(dir))) {
        return 0;
    }

    if (!face_direction(dir)) return 0;

    status = mm_forward(1);
    if (status == MM_BLOCKED) {
        record_side(mouse_x, mouse_y, dir, 1);
        mm_log("Blocked toward dir %d from (%d,%d)", dir, mouse_x, mouse_y);
        return 0;
    }
    if (status != MM_OK) {
        mm_log("ERROR: movement fault at (%d,%d)", mouse_x, mouse_y);
        return 0;
    }
    
    record_side(mouse_x, mouse_y, dir, 0);
    mouse_x = nx;
    mouse_y = ny;
    return 1;
}

//excploring all cells works in the DFS algo
static int explore_all_cells(void)
{

    /*
    per each lvel in the DFS we have the cells entry direction (where we entered from so we can backtrack safly), the next direction and the current depth of the stack, we start from depth zero and then recursivel go into each cell until we reach depth  again
    2- in this function we use the visted array so we can keep ttrack of all visited cells
    */
    int stack_entry_dir[MAX_CELLS];
    int next_dir[MAX_CELLS];
    int depth = 0;

    //for the first cell the entry direction is -1 
    stack_entry_dir[0] = -1;
    next_dir[0] = 0;
    visited[mouse_y][mouse_x] = 1;
    observe_current_cell();     //udated the known /walls array and colored the cell (we sensed the current cells right front left walls too)


    //for each cell we have next_dir, this is a variable to check all open path irections of the current cell start from 0 to 3
    while (depth >= 0) {
        int moved = 0;
        //this is if the user requrest the robot to reset 
        if (mm_reset_requested()) {
            mouse_x = 0;
            mouse_y = 0;
            heading = MM_NORTH;
            mm_reset_done();
            depth = 0;
            stack_entry_dir[0] = -1;
            next_dir[0] = 0;
            if (!visited[0][0]) visited[0][0] = 1;
            observe_current_cell();
        }

        while (next_dir[depth] < 4) {
            int dir = next_dir[depth]++;
            //we calculate the direction and calc the next cell we have not moved yet 
            int nx = mouse_x + dx[dir];
            int ny = mouse_y + dy[dir];

            /*we check f the next cell is in maze bounds or if we heading to a wall or not if any of the previosu we skip and check next diretion of the cell */
            if (!in_bounds(nx, ny)) continue;
            if (visited[ny][nx]) continue;
            if ((known[mouse_y][mouse_x] & MM_WALL(dir)) &&
                (walls[mouse_y][mouse_x] & MM_WALL(dir))) continue;
            //if not any of the above we move forward 1 so this means we go into a new cell and chek it again using the same function 
            if (move_one_cell(dir)) {
                if (depth + 1 >= MAX_CELLS) {
                    mm_log("ERROR: DFS stack capacity exceeded");
                    return 0;
                }
                ++depth;
                stack_entry_dir[depth] = dir;
                next_dir[depth] = 0;
                visited[mouse_y][mouse_x] = 1;
                observe_current_cell();
                moved = 1;
                break;
            }
            //if the moving did not happen then either a wall or error happeend so we check next direction 
        }

        if (moved) continue; //if moving happened we repeat
        
//else if not we backtrack
        if (depth == 0) break; //if we reached the furst cell then we explored everything 

        {//else e go back to th previous cell and continue fomr it's unvisited directions
            int return_dir = (stack_entry_dir[depth] + 2) % 4;
            if (!move_one_cell(return_dir)) {
                mm_log("ERROR: could not backtrack from (%d,%d)", mouse_x, mouse_y);
                return 0;
            }
            --depth;
        }
    }

    mm_log("Exploration complete; visited %d cells.",
           maze.width * maze.height);
    return mouse_x == 0 && mouse_y == 0;
}

static void bfs_distances(int dist[MAX_SIZE][MAX_SIZE], int multi_goal_sources)
/*
this function does BFS for every accesiple cell to know the value from it to the goal cells*/
{
    //BFS USES QUEUES
    int queue_x[MAX_CELLS]; // x coordinate of each cell and the next array is for y coordinate
    int queue_y[MAX_CELLS];
    int head = 0;
    int tail = 0;
    int x, y;

    for (y = 0; y < maze.height; ++y)
        for (x = 0; x < maze.width; ++x)
            dist[y][x] = -1;//initialize all ditances to -1

    if (multi_goal_sources) {       //if 1 then we calculate from goal if 0 we calc from the starting cell
        int gx, gy;
        for (gy = maze.goal_y0; gy <= maze.goal_y1; ++gy) {
            for (gx = maze.goal_x0; gx <= maze.goal_x1; ++gx) { //we go over each goal cell check if they valid and assign destance 00 to then, then add to queue
                if (in_bounds(gx, gy) && dist[gy][gx] == -1) {
                    dist[gy][gx] = 0;
                    queue_x[tail] = gx;
                    queue_y[tail] = gy;
                    ++tail;
                }
            }
        }
    } else { //start from starting cell
        dist[0][0] = 0;
        queue_x[tail] = 0;
        queue_y[tail] = 0;
        ++tail;
    }

    while (head < tail) {
        /*for erach cell we check the accessible eibors and assign a distance to it */
        int cx = queue_x[head];
        int cy = queue_y[head];
        int dir;
        ++head;

        for (dir = 0; dir < 4; ++dir) {
            int nx = cx + dx[dir];
            int ny = cy + dy[dir];
            uint8_t bit = MM_WALL(dir);
            
            if (!in_bounds(nx, ny)) continue;
            if (dist[ny][nx] != -1) continue;
            if (!(known[cy][cx] & bit)) continue;
            if (walls[cy][cx] & bit) continue;

            dist[ny][nx] = dist[cy][cx] + 1;
            if (tail < MAX_CELLS) {
                queue_x[tail] = nx;
                queue_y[tail] = ny;
                ++tail;
            }
        }
    }
}



static void display_distances(int dist[MAX_SIZE][MAX_SIZE])
{
    int x, y;
    char label[11];

    for (y = 0; y < maze.height; ++y) {
        for (x = 0; x < maze.width; ++x) {
            if (dist[y][x] < 0) {
                mm_label(x, y, NULL);
            } else {
                snprintf(label, sizeof(label), "%d", dist[y][x]);
                mm_label(x, y, label);
            }
        }
    }

    for (y = maze.goal_y0; y <= maze.goal_y1; ++y)
        for (x = maze.goal_x0; x <= maze.goal_x1; ++x)
            if (in_bounds(x, y)) mm_mark(x, y, MM_COLOR_GREEN);
}



static int travel_to_goal(int goal_dist[MAX_SIZE][MAX_SIZE])
{
    int steps = 0; //this is just tyo count how many moves teh mouse made it's a saftly limit
    while (!is_goal_cell(mouse_x, mouse_y)) {
        int dir;
        int chosen = -1;    //chosen direction for that cell
        int current_dist = goal_dist[mouse_y][mouse_x]; //how far from goal 

        if (current_dist <= 0) {
            mm_log("ERROR: no known route from (%d,%d) to goal", mouse_x, mouse_y);
            return 0;
        }

        for (dir = 0; dir < 4; ++dir) {
            int nx = mouse_x + dx[dir];
            int ny = mouse_y + dy[dir];
            if (!in_bounds(nx, ny)) continue;
            if (!(known[mouse_y][mouse_x] & MM_WALL(dir))) continue;
            if (walls[mouse_y][mouse_x] & MM_WALL(dir)) continue;
            if (goal_dist[ny][nx] == current_dist - 1) {//we go to the neighbor who's distance is 1 less than the current cell 
                chosen = dir;
                break;
            }
        }

        if (chosen < 0 || !move_one_cell(chosen)) {
            mm_log("ERROR: shortest-path movement failed");
            return 0;
        }
        ++steps;
        if (steps > MAX_CELLS) return 0;
    }
    mm_log("Reached goal at (%d,%d).", mouse_x, mouse_y);
    mm_mark(mouse_x, mouse_y, MM_COLOR_GREEN);
    return 1;
}

static int return_home(int start_dist[MAX_SIZE][MAX_SIZE])  //this function is alsmot identiacal to the previous one
{
    int steps = 0;

    while (mouse_x != 0 || mouse_y != 0) {
        int dir;
        int chosen = -1;
        int current_dist = start_dist[mouse_y][mouse_x];

        if (current_dist <= 0) {
            mm_log("ERROR: no known route back home from (%d,%d)", mouse_x, mouse_y);
            return 0;
        }

        for (dir = 0; dir < 4; ++dir) {
            int nx = mouse_x + dx[dir];
            int ny = mouse_y + dy[dir];
            if (!in_bounds(nx, ny)) continue;
            if (!(known[mouse_y][mouse_x] & MM_WALL(dir))) continue;
            if (walls[mouse_y][mouse_x] & MM_WALL(dir)) continue;
            if (start_dist[ny][nx] == current_dist - 1) {
                chosen = dir;
                break;
            }
        }

        if (chosen < 0 || !move_one_cell(chosen)) {
            mm_log("ERROR: return-path movement failed");
            return 0;
        }
        ++steps;
        if (steps > MAX_CELLS) return 0;
    }

    mm_log("Returned home successfully.");
    return 1;
}

static void dump_map(void)
{
    int x, y;

    mm_map_begin();
    for (y = 0; y < maze.height; ++y)
        for (x = 0; x < maze.width; ++x)
            mm_map_cell(x, y, walls[y][x], known[y][x]);
    mm_map_end();
}

void solver_run(void) //this function acts as the main function that controls the robot it's just a sequences of the previous rfuncitons 
{
    int goal_dist[MAX_SIZE][MAX_SIZE];  //stores the distacne of each cell to the goal 
    int start_dist[MAX_SIZE][MAX_SIZE]; //same but for starting cell

    memset(walls, 0, sizeof(walls));
    memset(known, 0, sizeof(known));
    memset(visited, 0, sizeof(visited));    //initialise all arrays with zeros

    mm_begin("T36");    
    maze = mm_maze();  

    if (maze.width < 1 || maze.height < 1 ||
        maze.width > MAX_SIZE || maze.height > MAX_SIZE ||
        maze.goal_x0 < 0 || maze.goal_y0 < 0 ||
        maze.goal_x1 >= maze.width || maze.goal_y1 >= maze.height ||
        maze.goal_x0 > maze.goal_x1 || maze.goal_y0 > maze.goal_y1) {
        mm_log("ERROR: invalid maze dimensions or goal coordinates");
        return;
    } //check wether the maze is avlid or not 

    mouse_x = 0;
    mouse_y = 0;
    heading = MM_NORTH; //initial state

    mm_log("M1 solver: maze %dx%d, goal (%d,%d)-(%d,%d)", maze.width, maze.height, maze.goal_x0, maze.goal_y0, maze.goal_x1, maze.goal_y1);

    /*
    first we initalise macq boundries
    2- explore all cells in maze
    3- find shortest bat hhto goal and display the values for each cell
    4- go to the goal using fastest path
    5- find shortest path to starting cell
    6- go to starting cell using the shortest path
    that's it */

    //step 1
    initialize_boundaries();
    //step2
    if (!explore_all_cells()) {
        mm_log("ERROR: exploration did not finish at the start");
        dump_map();
        return;
    }
    //step3
    bfs_distances(goal_dist, 1);
    display_distances(goal_dist);
    if (goal_dist[0][0] < 0) {
        mm_log("ERROR: goal is unreachable in the discovered map");
        dump_map();
        return;
    }
    mm_log("Shortest distance from start to nearest goal cell: %d",
           goal_dist[0][0]);
           //step4
    if (!travel_to_goal(goal_dist)) {
        dump_map();
        return;
    }
    //step5
    bfs_distances(start_dist, 0);
    //step 6
    if (!return_home(start_dist)) {
        dump_map();
        return;
    }
    mm_log("M1 run complete: explored, reached goal, and returned home.");
    dump_map();
}

/*
This Phase has been mostly made by Tareq (Algorithm Lead) and Oghaid (Motion Lead), the rest of the group members responsibility is to fully understand the functionality of the code
*/