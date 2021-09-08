#include <iostream>
#include <stdlib.h>
#include <SDL.h>
#include <math.h >
//#undef main

//To do:	Hybridisation
//			Dropout
//			Stop angle discontinuities

const int DELAY = 0;

const int SPACING = 50;
const int MAX_HEIGHT = 600;
const int MAX_WIDTH = 1200;


const float rDrones = 5;
const int N = (MAX_HEIGHT - SPACING) * (MAX_WIDTH- SPACING) / (SPACING * SPACING);

//Network parameters:
const int width = 10, depth = 4, os = 2, N_CLOSE = 5, is = 4 + N_CLOSE*4; //'is' is the number of inputs, 'os' is the number of outputs, N_CLOSE is the number of drones which each drone senses;

const int N_PARA = is * (width + 1) + os * width + depth * (width + 1);
const int mutation_rate = 1;

int DEAD_FROM_CRASHES;
int DEAD_FROM_OUT_OF_BOUNDS;

typedef struct drone
{
	bool alive;
	bool predator;
	int score;

	//Inputs for net:
	float xpos;
	float ypos;
	float angle;
	int closest[N_CLOSE];

	//Outputs from net:
	float ang_v;
	float speed;
	float xvel;
	float yvel;

	//Parameters: 
	float sw[width][is + 1]; //Starting weights, i.e. layer 0
	float iw[depth][width][width + 1]; //Inner weights 
	float fw[os][width + 1]; //Final weights
} drone;

drone* DRONES = (drone*) malloc((N)* sizeof(drone));

void draw(SDL_Renderer* Renderer, int xpos, int ypos, float angle, bool type) {
	//Setting the draw color, and the small angle of the triangles:
	//SDL_SetRenderDrawColor(Renderer, rand()&255, rand() & 255, rand() & 255, SDL_ALPHA_OPAQUE);
	if (type)
		SDL_SetRenderDrawColor(Renderer, 255, 0, 0, SDL_ALPHA_OPAQUE);
	else 
		SDL_SetRenderDrawColor(Renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);

	float nose_a = 2.6;

	//Drawing the lines we want. (-1, 0), (0, 1), (1, 0)
	int x1 = xpos + rDrones * cos(angle), x2 = xpos + rDrones * cos(angle + nose_a), x3 = xpos + rDrones * cos(angle - nose_a);
	int y1 = ypos - rDrones * sin(angle), y2 = ypos - rDrones * sin(angle + nose_a), y3 = ypos - rDrones * sin(angle - nose_a);

	SDL_RenderDrawLine(Renderer, x1, y1, x2, y2);
	SDL_RenderDrawLine(Renderer, x2, y2, x3, y3);
	SDL_RenderDrawLine(Renderer, x3, y3, x1, y1);
}

void drawConnections(SDL_Renderer* Renderer, int n)
{
	for (int i = 0; i < N_CLOSE; i++) 
	{
		//SDL_SetRenderDrawColor(Renderer, n * (255 / N), i * (255/N_CLOSE), 255 / (i + 1), 0);
		SDL_RenderDrawLine(Renderer, DRONES[n].xpos, DRONES[n].ypos, DRONES[DRONES[n].closest[i]].xpos, DRONES[DRONES[n].closest[i]].ypos);
	}
}

void update(SDL_Renderer* renderer) {
	SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
	SDL_RenderClear(renderer);

	for (int n = 0; n < N; n++)
	{
		//Check if dead: 
		if (DRONES[n].alive == false)
			continue;

		if (DRONES[n].xpos > MAX_WIDTH || DRONES[n].xpos < 0) 
		{
			DRONES[n].alive = false;
			DEAD_FROM_OUT_OF_BOUNDS++;
			continue;
		}

		if (DRONES[n].ypos > MAX_HEIGHT || DRONES[n].ypos < 0) 
		{
			DRONES[n].alive = false;
			continue;
		}

		float c_dists[N_CLOSE];

		for (int i = 0; i < N_CLOSE; i++) {
			c_dists[i] = MAX_HEIGHT* MAX_HEIGHT + MAX_WIDTH * MAX_WIDTH;
		}

		for (int i = 0; i < N; i++)
		{
			if (DRONES[i].alive and (i != n))
			{
				float xdist = DRONES[i].xpos - DRONES[n].xpos;
				float ydist = DRONES[i].ypos - DRONES[n].ypos;
				float dist2 = xdist * xdist + ydist * ydist;
				
				//For this version, only 'predator' drones result in a crash:
				if ((dist2 < rDrones * rDrones *4) and (DRONES[n].predator == false) and (DRONES[i].predator)) {
					DRONES[n].alive = false;
					DRONES[i].score ++;
					DEAD_FROM_CRASHES ++;
					break;
				}

				for (int j = 0; j < N_CLOSE; j++)
				{
					if (dist2 < c_dists[j])
					{
						for (int e = N_CLOSE - 1; e > j - 1; e--)
						{
							c_dists[e] = c_dists[e - 1];
							DRONES[n].closest[e] = DRONES[n].closest[e-1];
						}
						c_dists[j] = dist2;
						DRONES[n].closest[j] = i;
						break;
					}
				}
			}
		}
		
		if (!DRONES[n].alive) continue;

		//Draw them:
		draw(renderer, DRONES[n].xpos, DRONES[n].ypos, DRONES[n].angle, DRONES[n].predator);
		drawConnections(renderer, n);

		//Update Positions:
		DRONES[n].angle = DRONES[n].angle + DRONES[n].ang_v;
		DRONES[n].angle = fmod(DRONES[n].angle, 2*3.14159);
		DRONES[n].xvel = cos(DRONES[n].angle) * DRONES[n].speed;
		DRONES[n].yvel = -sin(DRONES[n].angle) * DRONES[n].speed;
		DRONES[n].xpos = DRONES[n].xpos + DRONES[n].xvel;
		DRONES[n].ypos = DRONES[n].ypos + DRONES[n].yvel;

		//Updating parameters for the next cycle:
		float A[width];
		float B[width];
		float * r  = A;
		float * rn = B;
		float * rt;

		//Starting parameters:

		for (int i = 0; i < width; i++)
		{
			r[i] = DRONES[n].sw[i][0] + DRONES[n].sw[i][1] * DRONES[n].xvel + DRONES[n].sw[i][2] * DRONES[n].yvel + DRONES[n].sw[i][3] * DRONES[n].xpos/MAX_WIDTH + DRONES[n].sw[i][4] * DRONES[n].ypos/MAX_HEIGHT;
			//closest drones:
			for (int c = 0; c < N_CLOSE; c++) {
				r[i] += DRONES[DRONES[n].closest[c]].xvel * DRONES[n].sw[i][5+c];
				r[i] += DRONES[DRONES[n].closest[c]].yvel *DRONES[n].sw[i][6+c];
				r[i] += DRONES[DRONES[n].closest[c]].xpos * DRONES[n].sw[i][7+c] / MAX_WIDTH;
				r[i] += DRONES[DRONES[n].closest[c]].ypos * DRONES[n].sw[i][8+c] / MAX_HEIGHT;
			}
			//Apply Activation Function
			r[i] = (r[i] > 0) ? r[i] : 0;
		}

		//Inner Nodes
		for (int d = 0; d < depth; d++) //Repeat for each depth
		{
			for (int i = 0; i < width; i++) //Repeat for each node
			{
				rn[i] = DRONES[n].iw[d][i][0]; // Add a bias
				for (int j = 1; j < (width + 1); j++) //Add the sum of each previous node, multiplied by weight. 
					rn[i] = rn[i] + r[j - 1] * DRONES[n].iw[d][i][j];
				//Apply Activation Function
				r[i] = (r[i] > 0) ? r[i] : 0;
			}

			//Switch pointers for r and rn
			rt = r;
			r = rn;
			rn = rt;
		}

		//Taking output from the net:
		float newVals[os];
		for (int i = 0; i < os; i++)
		{
			newVals[i] = DRONES[n].fw[i][0];
			for (int j = 1; j < (width + 1); j++)
				newVals[i] = newVals[i] + r[j-1] * DRONES[n].fw[i][j];
		}

		//Applying outputs: (HARDCODED LOL) Different Activation Function
		DRONES[n].speed = 4 + 2*tanhf(newVals[0]);
		DRONES[n].ang_v = 0.02*tanhf(newVals[1]);

	}

	//Update the Renderer.

	SDL_RenderPresent(renderer);
	SDL_Delay(DELAY);
}

void mutate(int n) 
{
	//sw
	for (int i = 0; i < width; i++)
	{
		for (int j = 0; j < (is + 1); j++)
			if(rand() % N_PARA < mutation_rate) DRONES[n].sw[i][j] += sin(rand());
	}

	//iw
	for (int d = 0; d < depth; d++)
		for (int i = 0; i < width; i++)
		{
			//DRONES[n].iw[d][i][0] = 0;
			for (int j = 0; j < width + 1; j++)
				if (rand() % N_PARA < mutation_rate) DRONES[n].iw[d][i][j] += sin(rand());
		}
	//fw
	for (int i = 0; i < os; i++)
	{
		//DRONES[n].fw[i][0] = 0;
		for (int j = 0; j < width + 1; j++)
			if (rand() % N_PARA < mutation_rate) DRONES[n].fw[i][j] += sin(rand());
	}
}

int runGeneration(SDL_Renderer* renderer)
{
	for (int n = 0; n < N; n++)
	{

		DRONES[n].alive = true; 
		DRONES[n].xpos = SPACING + (n * SPACING) % (MAX_WIDTH - SPACING);
		DRONES[n].ypos = SPACING + (n * SPACING) / (MAX_WIDTH - SPACING) * SPACING;
		DRONES[n].speed = 0.1;
		DRONES[n].ang_v = 0;
	}

	int p;
	float temp_pos;

	//Swap and shuffle positions:
	for (int n = 0; n < N; n++) {
		p = rand() % N;
		temp_pos = DRONES[n].xpos;
		DRONES[n].xpos = DRONES[p].xpos;
		DRONES[p].xpos = temp_pos;

		temp_pos = DRONES[n].ypos;
		DRONES[n].ypos = DRONES[p].xpos;
		DRONES[p].ypos = temp_pos;
	}

	//Set angles:
	for (int n = 0; n < N; n++)
	{
		//Set Angle
		DRONES[n].angle = float(rand() %3); //(DRONES[n].xpos < MAX_WIDTH / 2) ? 0 + sin(rand()) : 3.14 + sin(rand());
		//DRAWING
	}

	//Run them:
	for (int t = 0; t < 300; t++) 
	{
		update(renderer);
	}

	for (int n = 0; n < N; n++) if (DRONES[n].alive) DRONES[n].score++;

    return 0;
}

void initialiseDrones() 
{
	for (int n = 0; n < N; n++)
	{
		//Number of predators = N/2:
		if (n % 2 == 0)
			DRONES[n].predator = true;
		else 
			DRONES[n].predator = false;
		DRONES[n].score = 0;
		//sw
		for (int i = 0; i < width; i++)
		{
			for (int j = 0; j < (is + 1); j++)
				DRONES[n].sw[i][j] = sin(rand());
		}

		//iw
		for (int d = 0; d < depth; d++)
			for (int i = 0; i < width; i++)
			{
				//DRONES[n].iw[d][i][0] = 0;
				for (int j = 0; j < width + 1; j++)
					DRONES[n].iw[d][i][j] = sin(rand());
			}
		//fw
		for (int i = 0; i < os; i++)
		{
			//DRONES[n].fw[i][0] = 0;
			for (int j = 0; j < width + 1; j++)
				DRONES[n].fw[i][j] = sin(rand());
		}
	}
}

void repopulate()
{
	int totalSurvival = 0;
	int predTotal = 0;
	for (int n = 0; n < N; n++) if (!DRONES[n].predator) totalSurvival += DRONES[n].score;
	for (int n = 0; n < N; n++) if ( DRONES[n].predator) predTotal     += DRONES[n].score;

	//Collect the living:
	//for (int n = 0; n < N; n++)
	//{
	//	if (DRONES[n].alive)
	//	{
	//		livingArr[nAlive] = n;
	//		nAlive++;
	//	}
	//}
	
	printf("Number alive: %d\n\tDead from crashes: %d\n\tDead from boundary: %d\n", totalSurvival, DEAD_FROM_CRASHES, DEAD_FROM_OUT_OF_BOUNDS);
	DEAD_FROM_CRASHES = 0;
	DEAD_FROM_OUT_OF_BOUNDS = 0;
	if (totalSurvival == 0) totalSurvival++; //This is fucking ugly but whatever 
	if (predTotal == 0) predTotal++;

	for (int n = 0; n < N; n++) 
	{
		if (DRONES[n].alive)
			continue;

		//Select the new drone:
		int sel = 0;
		int count = 0;
		int p = 0;
		sel = rand() % (DRONES[n].predator ? predTotal : totalSurvival);
		while (p < N - 1)
		{
			if (DRONES[p].predator == DRONES[n].predator)
			{
				count += DRONES[p].score;
				if (count > sel) break;
			}
			p++;
		}

		//sw
		for (int i = 0; i < width; i++)
		{
			for (int j = 0; j < (is + 1); j++)
				DRONES[n].sw[i][j] = DRONES[p].sw[i][j];
		}

		//iw
		for (int d = 0; d < depth; d++)
			for (int i = 0; i < width; i++)
			{
				for (int j = 0; j < width + 1; j++)
					DRONES[n].iw[d][i][j] = DRONES[p].iw[d][i][j];
			}
		//fw
		for (int i = 0; i < os; i++)
		{
			//DRONES[n].fw[i][0] = 0;
			for (int j = 0; j < width + 1; j++)
				DRONES[n].fw[i][j] = DRONES[p].fw[i][j];
		}
		mutate(n);

	}

	for (int n = 0; n < N; n++)
		DRONES[n].score = 0;
};

void saveDrones()
{
	char fname[100];
	snprintf(fname, 100,"Ndrones = %d, w = %d, d = %d Nclose = %d.csv", N, width, depth, N_CLOSE);
	#pragma warning(suppress : 4996)
	FILE* out_file = fopen(fname, "w"); // write only 

	for (int n = 0; n < N; n++)
	{
		for (int i = 0; i < width; i++)
		{
			for (int j = 0; j < (is + 1); j++)
				fprintf(out_file, "%f,", DRONES[n].sw[i][j]);
		}
		for (int d = 0; d < depth; d++)
			for (int i = 0; i < width; i++)
			{
				for (int j = 0; j < width + 1; j++)
					fprintf(out_file, "%f,", DRONES[n].iw[d][i][j]);
			}
		for (int i = 0; i < os; i++)
		{
			for (int j = 0; j < width + 1; j++)
				fprintf(out_file, "%f,", DRONES[n].fw[i][j]);
		}
		fprintf(out_file, "\n");
	}
	fclose(out_file);
}

int main(int argc, char* argv[])
{
	//Initialising: 
	//SDL_Init(SDL_INIT_VIDEO);
	SDL_Window* window = SDL_CreateWindow("DISPLAY", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, MAX_WIDTH, MAX_HEIGHT, SDL_WINDOW_SHOWN);
	SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, 0);
	SDL_SetRenderDrawColor(renderer, 0, 0, 60, 0);
	SDL_RenderClear(renderer);
	SDL_RenderPresent(renderer);

	time_t t;
	srand((unsigned)time(&t));


	initialiseDrones();

	for (int ngen = 0; ngen < 50; ngen++)
	{

		runGeneration(renderer);
		saveDrones();
		//DRAW()
		SDL_SetRenderDrawColor(renderer, 0, 0, 60, 0);
		//SDL_RenderClear(renderer);
		SDL_RenderPresent(renderer);

		repopulate();
	}

	saveDrones();
	return 0;

	//Ending:
	SDL_Delay(400);

	SDL_DestroyWindow(window);
	SDL_Quit();
}