#include<stdio.h>
#include<graphics.h>
#include<math.h>
#include <stdlib.h>
#include <time.h>
#define M_PI 3.14159265358979323846
#define MAX_STATES 1000

static const float Dt = 0.1f; // Intervale de temps entre chaque mise à jour de la position et l'orientation
static const float D = 0.05f; // Distance entre les roues D = 0.05 m
static const float Rr = 0.05f;// Rayon du robot Rr = 0.05 m
static const float R0 = 0.02f;// Rayon d'une roue R0 = 0.02 m
static const float w0Max = 10.0f;// Vitesse angulaire maximum des roues w0Max = 10 rad/s
static const float Dw0Max = 2.0f;// Acceleration angulaire maximum des roues Dw0Max = 2 rad/s

// Structure to represent an Robot
typedef struct {
    float x=400,y=200,r=50;
    float Dr;//Distance parcourue pendant Dt en m
    float Dalpha;//Reorientation pendant Dt en rad
    float wg;//Vitesse angulaire de la roue gauche en rad/s
    float wd;//Vitesse angulaire de la roue droite en rad/s
    float Dg;//Distance parcourue par la roue gauche pendant Dt en m
    float Dd;//Distance parcourue par la roue droite pendant Dt en m
    float Rc;//Rayon de courbure de la trajectoire du robot en m
    float Dx, Dy;//Distance instantanee du robot pendant Dt de x et y en pixel/s
} Robot;
// Structure to represent an Goal
typedef struct
{
    int x;
    int y;
    int radius;
} Goal;
// Structure to represent an obstacle
typedef struct
{
    int x;
    int y;
    int radius;
} Obstacle;

// Robot functions

float Robot_getDalpha(Robot* robot) {
    return robot->Dalpha;
}

void Robot_updateDr(Robot* robot) {
    robot->Dr = (robot->Dg + robot->Dd) / 2;
}

void Robot_updateDalpha(Robot* robot) {
    robot->Dalpha += (robot->Dg - robot->Dd) / D;
}

void Robot_updateDg(Robot* robot) {
    robot->Dg = robot->wg * Dt * R0;
}

void Robot_updateDd(Robot* robot) {
    robot->Dd = robot->wd * Dt * R0;
}

void Robot_updateDx(Robot* robot) {
    robot->Dx = robot->Dr * cos(robot->Dalpha);
}

void Robot_updateDy(Robot* robot) {
    robot->Dy = robot->Dr * sin(robot->Dalpha);
}


float Robot_getX(Robot* robot) {
    return robot->x;
}

void Robot_setX(Robot* robot, float x) {
    robot->x = x;
}

float Robot_getY(Robot* robot) {
    return robot->y;
}

void Robot_setY(Robot* robot, float y) {
    robot->y = y;
}

float Robot_getR(Robot* robot) {
    return robot->r;
}

void Robot_setR(Robot* robot, float r) {
    robot->r = r;
}

// Check if the robot is colliding with an obstacle and return the direction to avoid the obstacle
int check_collision(Robot robot, Obstacle obstacle, float *dx, float *dy)
{
    // Calculate the distance between the center of the robot and the center of the obstacle
    float distance = sqrt((robot.x - obstacle.x) * (robot.x - obstacle.x) +
                         (robot.y - obstacle.y) * (robot.y - obstacle.y)) - 20;

    // Return 1 if the distance is less than the sum of the radii, 0 otherwise
    if (distance < robot.r + obstacle.radius)
    {
        // Calculate the angle between the center of the robot and the center of the obstacle
        float angle = atan2(obstacle.y - robot.y, obstacle.x - robot.x);

        // Set the new direction of the robot to move around the obstacle
        *dx = 0.5 * cos(angle + M_PI / 2) ;
        *dy = 0.5 * sin(angle + M_PI / 2) ;
        return 1;
    }
    else
    {
        return 0;
    }
}


void Robot_updateDalpha(Robot* robot, float Dalpha) {
    //robot->Dalpha = Dalpha ;
    // Calculate the difference between the current value of Dalpha and the target value
    float delta = Dalpha - robot->Dalpha;
    // Use a smoothing function to gradually update the value of Dalpha over time
    robot->Dalpha += 0.1 * delta;
}

void avoidCollisionWithObstacle(Robot* robot, Obstacle* obstacle) {
    // Check if the robot is colliding with the obstacle
    float dx, dy;
    if (check_collision(*robot, *obstacle, &dx, &dy)) {
        // The robot is colliding with the obstacle, so update its position to avoid the collision
        robot->x += dx * 5; // Move the robot at half the speed to make it move slowly
        robot->y += dy * 5;
        Robot_updateDalpha(robot, atan2(dy, dx)); // Update the rotation angle of the robot to match the direction of the rotation
    }
}

// function that makes the robot move towards the goal:
void move_towards_goal(Robot *robot, Goal *goal)
{
    // Calculate the difference between the x and y coordinates of the goal and the robot
    float diff_x = goal->x - robot->x;
    float diff_y = goal->y - robot->y;

    // Update the dx and dy values of the robot based on the difference between the x and y coordinates
    if (abs(diff_x) > abs(diff_y))
    {
        robot->Dx = (diff_x > 0) ? 5 : -5;
        robot->Dy = (diff_y > 0) ? 5 * diff_y / abs(diff_x) : -5 * diff_y / abs(diff_x);
    }
    else
    {
        robot->Dx = (diff_x > 0) ? 5 * diff_x / abs(diff_y) : -5 * diff_x / abs(diff_y);
        robot->Dy = (diff_y > 0) ? 5 : -5;
    }
}
// Check if the robot is colliding with a goal and return 1 if it is, 0 otherwise
int reached_goal(Robot robot, Goal goal)
{
    // Calculate the distance between the center of the robot and the center of the goal
    float distance = sqrt((robot.x - goal.x) * (robot.x - goal.x) +
                         (robot.y - goal.y) * (robot.y - goal.y));

    // Return 1 if the distance is less than the sum of the radii, 0 otherwise
    return distance < robot.r + goal.radius;
}

void AutoMove(Robot* robot, Goal goal) {
	// Compute the angle between the robot and the goal.
    float angle = atan2(goal.y - robot->y, goal.x - robot->x);
    Robot_updateDalpha(robot, angle);

    // Update the position of the robot based on its orientation and speed.
    robot->x += 2.5 * cos(robot->Dalpha);
    robot->y += 2.5 * sin(robot->Dalpha);
    
}
// Function to save the state of the robot to a file
int num_states_saved = 0;
void save_robot_state(Robot robot) {
    
    FILE *fp = fopen("state.pts", "a");
    if (fp == NULL) {
        printf("Error opening file!\n");
        return;
    }

    if (num_states_saved < MAX_STATES) {
        // Write the state to the file in the specified format
        fprintf(fp, "< %d > < %f > < %f > < %f > < %f >\n", num_states_saved, robot.x, robot.y, robot.wg, robot.wg);
        num_states_saved++;
    } else {
        // Go to the beginning of the file
        rewind(fp);

        // Find the oldest state
        int i;
        for (i = 0; i < MAX_STATES - 1; i++) {
            fscanf(fp, "%*d %*f %*f %*f %*f %*f %*f\n");
        }

        // Overwrite the oldest state with the new state
        fprintf(fp, "< %d > < %f > < %f > < %f > < %f >\n", num_states_saved, robot.x, robot.y, robot.wg, robot.wg);
    }
    fclose(fp);
}
//instance of Robot
Robot robot;
int main(void){
	
	float scale=1.0f;
	initwindow(800,800,"Auto Robot"); 

	float alpha;
	// Initialize the obstacle
    	// Load obstacles from .obs file
    FILE *obstacle_file = fopen("obstacles.obs", "r");
    if (obstacle_file == NULL) {
        printf("Error: Could not open obstacles file.\n");
        return 1;
    }

    int num_obstacles;
    fscanf(obstacle_file, "%d", &num_obstacles);

    Obstacle obstacles[num_obstacles];

    for (int i = 0; i <= num_obstacles; i++) {
        int x, y, r;
        fscanf(obstacle_file, "%d %d %d", &x, &y, &r);
        obstacles[i] = (Obstacle) {x, y, r};
    }

    fclose(obstacle_file);
    //Goal
    Goal goal;
    goal.radius = 50;
    srand(time(0)); // Choose a different seed at each execution of the program
    goal.x = 550;
    goal.y = 400;
	while(1){
			
			alpha = Robot_getDalpha(&robot);
			
	        cleardevice();
	        setlinestyle(0,0,2);
	        //
	        // Draw the obstacles
	        setcolor(RED);
	        for (int i = 0; i <= num_obstacles; i++)
	        {
	            circle(obstacles[i].x, obstacles[i].y, obstacles[i].radius);
	            setfillstyle(SOLID_FILL, RED);
	            floodfill(obstacles[i].x, obstacles[i].y, BLACK);
	        }
	        
	        // Draw the goal
	        setcolor(GREEN);
	        circle(goal.x, goal.y, goal.radius);
	        setfillstyle(SOLID_FILL, GREEN);
	        floodfill(goal.x, goal.y, GREEN);
	        
	        /**
	          *---------------------- Draw The Robot ----------------------------
			***/
			setcolor(WHITE);
	        circle(Robot_getX(&robot),Robot_getY(&robot),Robot_getR(&robot));
	        
	        /**
	          *----------------------Draw TRIANGLE Using trigonometric circle----------------------------
			***/
			
	        line(Robot_getX(&robot) + (int)Robot_getR(&robot)*cos(alpha - (M_PI/2)),
				Robot_getY(&robot)+ (int)Robot_getR(&robot)*sin(alpha - (M_PI/2)), 
				Robot_getX(&robot) + (int)Robot_getR(&robot)*cos(alpha),
				Robot_getY(&robot)+ (int)Robot_getR(&robot)*sin(alpha));
			
	        line(Robot_getX(&robot) + (int)Robot_getR(&robot)*cos(alpha), 
				Robot_getY(&robot)+ (int)Robot_getR(&robot)*sin(alpha), 
				Robot_getX(&robot) + (int)Robot_getR(&robot)*cos(alpha + (M_PI/2)), 
				Robot_getY(&robot)+ (int)Robot_getR(&robot)*sin(alpha + (M_PI/2)));
			
	        line(Robot_getX(&robot) + (int)Robot_getR(&robot)*cos(alpha - (M_PI/2)), 
				Robot_getY(&robot)+ (int)Robot_getR(&robot)*sin(alpha - (M_PI/2)), 
				Robot_getX(&robot) + (int)Robot_getR(&robot)*cos(alpha + (M_PI/2)), 
				Robot_getY(&robot)+ (int)Robot_getR(&robot)*sin(alpha + (M_PI/2)));
			/**
	          *----------------------Draw wheels----------------------------
			***/
			
			// Calculate the center point of the existing line segment
			float center_x = (Robot_getX(&robot) + (int)Robot_getR(&robot)*cos(alpha - M_PI/2) + 
			                  Robot_getX(&robot) + (int)Robot_getR(&robot)*cos(alpha + M_PI/2)) / 2;
			float center_y = (Robot_getY(&robot) + (int)Robot_getR(&robot)*sin(alpha - M_PI/2) + 
			                  Robot_getY(&robot) + (int)Robot_getR(&robot)*sin(alpha + M_PI/2)) / 2;
			
			// Calculate the endpoints of the horizontal line segment on the top of the existing line segment
			float top_x1 = center_x + (int)Robot_getR(&robot)/4*cos(alpha) + 15*cos(alpha - M_PI/2);
			float top_y1 = center_y + (int)Robot_getR(&robot)/4*sin(alpha) + 15*sin(alpha - M_PI/2);
			float top_x2 = center_x + (int)Robot_getR(&robot)/4*cos(alpha - M_PI) + 15*cos(alpha - M_PI/2);
			float top_y2 = center_y + (int)Robot_getR(&robot)/4*sin(alpha - M_PI) + 15*sin(alpha - M_PI/2);
			
			// Calculate the endpoints of the horizontal line segment on the bottom of the existing line segment
			float bottom_x1 = center_x - (int)Robot_getR(&robot)/4*cos(alpha) - 15*cos(alpha - M_PI/2);
			float bottom_y1 = center_y - (int)Robot_getR(&robot)/4*sin(alpha) - 15*sin(alpha - M_PI/2);
			float bottom_x2 = center_x - (int)Robot_getR(&robot)/4*cos(alpha - M_PI) - 15*cos(alpha - M_PI/2);
			float bottom_y2 = center_y - (int)Robot_getR(&robot)/4*sin(alpha - M_PI) - 15*sin(alpha - M_PI/2);
	
			// Draw horizontal line segment on the top of the existing line segment
			line(top_x1, top_y1, top_x2, top_y2);
	
			// Draw horizontal line segment on the bottom of the existing line segment
			line(bottom_x1, bottom_y1, bottom_x2, bottom_y2);
			
			
			// Check if the robot is colliding with any of the obstacles and avoid them
	        int collision = 0;
	        float nwg = 0.0f;
			float nwd = 0.0f;
	        for (int i = 0; i < num_obstacles; i++)
	        {
	            collision |= check_collision(robot, obstacles[i], &robot.Dx, &robot.Dy);
	            avoidCollisionWithObstacle(&robot, &obstacles[i]);
	                // Check if the goal is below the robot
    if (goal.y > robot.y) {
        // Move the robot down
        if (nwg + 0.5 <= w0Max) {
            nwg += 0.5;
            robot.wg = nwg;
    		robot.wd = nwd;
        }
        if (nwd + 0.5 <= w0Max) {
            nwd += 0.5;
            robot.wg = nwg;
    		robot.wd = nwd;
        }
    }
    // Check if the goal is above the robot
    else if (goal.y < robot.y) {
        // Move the robot up
        if (nwg - 0.5 >= -w0Max) {
            nwg -= 0.5;
            robot.wg = nwg;
    		robot.wd = nwd;
        }
        if (nwd - 0.5 >= -w0Max) {
            nwd -= 0.5;
            robot.wg = nwg;
    		robot.wd = nwd;
        }
    }
    // Check if the goal is to the left of the robot
    else if (goal.x < robot.x) {
        // Move the robot left
        if (nwg - 0.5 >= -w0Max && nwd + 0.5 <= w0Max) {
            nwg -= 0.5;
            nwd += 0.5;
            robot.wg = nwg;
    		robot.wd = nwd;
        }
    }
    // Check if the goal is to the right of the robot
    else if (goal.x > robot.x) {
        // Move the robot right
        if (nwg - 0.5 >= -w0Max && nwd + 0.5 <= w0Max) {
            nwg += 0.5;
            nwd -= 0.5;
            robot.wg = nwg;
    		robot.wd = nwd;
        }
    }
    // If the goal is not in any of these four positions, the robot is already at the goal
    else {
        // Stop the robot
        if (nwg > nwd) {
            nwg -= 0.25;
            nwd += 0.25;
            robot.wg = nwg;
    		robot.wd = nwd;
        }
        if (nwg < nwd) {
            nwd -= 0.25;
            nwg += 0.25;
            robot.wg = nwg;
    		robot.wd = nwd;
        }
    }
	            // Save the State of the Robot :
				save_robot_state(robot); 
	        }
	        
	        if(!collision)
			{
				// Check if the goal is below the robot
    if (goal.y > robot.y) {
        // Move the robot down
        if (nwg + 0.5 <= w0Max) {
            nwg += 0.5;
            robot.wg = nwg;
    		robot.wd = nwd;
        }
        if (nwd + 0.5 <= w0Max) {
            nwd += 0.5;
            robot.wg = nwg;
    		robot.wd = nwd;
        }
    }
    // Check if the goal is above the robot
    else if (goal.y < robot.y) {
        // Move the robot up
        if (nwg - 0.5 >= -w0Max) {
            nwg -= 0.5;
            robot.wg = nwg;
    		robot.wd = nwd;
        }
        if (nwd - 0.5 >= -w0Max) {
            nwd -= 0.5;
            robot.wg = nwg;
    		robot.wd = nwd;
        }
    }
    // Check if the goal is to the left of the robot
    else if (goal.x < robot.x) {
        // Move the robot left
        if (nwg - 0.5 >= -w0Max && nwd + 0.5 <= w0Max) {
            nwg -= 0.5;
            nwd += 0.5;
            robot.wg = nwg;
    		robot.wd = nwd;
        }
    }
    // Check if the goal is to the right of the robot
    else if (goal.x > robot.x) {
        // Move the robot right
        if (nwg - 0.5 >= -w0Max && nwd + 0.5 <= w0Max) {
            nwg += 0.5;
            nwd -= 0.5;
            robot.wg = nwg;
    		robot.wd = nwd;
        }
    }
    // If the goal is not in any of these four positions, the robot is already at the goal
    else {
        // Stop the robot
        if (nwg > nwd) {
            nwg -= 0.25;
            nwd += 0.25;
            robot.wg = nwg;
    		robot.wd = nwd;
        }
        if (nwg < nwd) {
            nwd -= 0.25;
            nwg += 0.25;
            robot.wg = nwg;
    		robot.wd = nwd;
        }
    }
	        	AutoMove(&robot, goal);
	        	// Save the State of the Robot :
				save_robot_state(robot);
			}
	        
	        
				// Check if the robot is at the goal
		        if (reached_goal(robot, goal))
		        {
					 // If the robot is at the goal, create a new goal within the screen and avoid the edges and obstacles
		            goal.radius = 50;
		            int new_x, new_y;
		            do
		            {
		                // Generate random x and y coordinates within the range of the screen
		                new_x = (int)((rand() / (double)RAND_MAX) * (getmaxx() - 2 * goal.radius) + goal.radius);
		                new_y = (int)((rand() / (double)RAND_MAX) * (getmaxy() - 2 * goal.radius) + goal.radius);
		
		                // Check the distance between the new goal and all obstacles
		                int collision = 0;
		                for (int i = 0; i < num_obstacles; i++)
		                {
		                    float distance = sqrt((new_x - obstacles[i].x) * (new_x - obstacles[i].x) +
		                                         (new_y - obstacles[i].y) * (new_y - obstacles[i].y));
		                    if (distance < goal.radius + obstacles[i].radius)
		                    {
		                        collision = 1;
		                        break;
		                    }
		                }
		
		                // If the new goal is not colliding with any obstacles, exit the loop
		                if (!collision)
		                {
		                    break;
		                }
		            } while (1);
		
		            // Set the new x and y coordinates for the goal
		            goal.x = new_x;
		            goal.y = new_y;
		        }

			// delay
			delay(100);
		}
    
	    closegraph();
		return 0;
	
}

