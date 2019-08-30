#include <iostream>
#include <vector>
#include <numeric>

using std::cout;
using std::endl;

				
const double cells = 20; 																//cells = number of doors 


double motionProbability(bool motion, int previous_position, int current_poistion)
{
	double motion_p = 0;																//variable to return motion probablity; 
	int distance = 0;																	//variable to hold difference in position

	//Temporary variables to hold inputs 
	bool tempmotion = motion;
	int tempprev = previous_position ;
	int tempcurr = current_poistion ;

	//if motion input is 0 
	if (tempmotion == false)
	{     
		distance = tempcurr - tempprev;													//calculates distance moved by robot
		if (distance == 0)																//checks to see if moved distance is 0
		{
			motion_p = 1.0;
    	}
    else 																				//other conditions probablity is 0
    {
			motion_p = 0.0;
    }

	}

	//if motion input is 1
	else if (tempmotion == true)
	{
		distance = tempcurr - tempprev;
		
		//possible cases for the robot for a 20 cell universe 
		switch (distance)
		{
			case 1 :
			motion_p = 0.7;
			break;

			case 2:
			motion_p = 0.3;
			break;

			case -19:																	//this condition is for the wrapped vector
			motion_p = 0.7 ;
			break;

			case -18:																	//this condition is for the wrapped vector
			motion_p = 0.3;
			break;       
    	}

	}  
  return motion_p;
  
}

double observationProbability(bool observation, int position)
{

	double obs_p;																		//variable to hold and return observation probability

	//Temporary variables to hold inputs 
	bool tempobservation = observation;
	int tempposition= position; 


 	//conditions when there is a door
	if (tempposition == 2| tempposition == 4 |tempposition == 7 )
	{
		if (tempobservation == false)													//checks to see if door detection failed
		{
			obs_p = 0.2;
		}
		else if (tempobservation ==true)												//checks to see if door detection success
		{
			obs_p = 0.8;		
		}
		else																			//defualts condition is zero
		{		
			obs_p = 0.0;	
		}


	}
	//conditions when there is no door
	else if (tempposition != 2| tempposition != 4 |tempposition != 7 )
	{
		if (tempobservation == false)													//checks to see if door detection failed
		{
			obs_p = 0.9;
		}
		else if (tempobservation ==true)												//checks to see if door detection success
		{
			obs_p = 0.1;		
		}
		else
		{
			obs_p = 0.0;																//defaults condition is zero
		}

	}
	
  
	return obs_p;
}

void normaliseState(std::vector<double>& state)
{
	double sum = 0; 																	// variable that holds sum of all probablities
	
	//loops that iterates through vector and sums and adds back into sum
	for (auto it = state.begin() ; it != state.end() ; it++)
	{
		sum += *it;
	}

	//Loop that iterates and scales down and assigns back to vector	
	for (int i = 0 ; i < sum ; i ++)
	{
		state [i] /= sum;
	}
	
}
	

void initialiseState(std::vector<double>& state)
{
	// Fill the state variable with initial probabilities
  
	double initial_prob = 1.0/cells;  													//initialise variable with the probability per cell

	state.assign (cells ,initial_prob);													//Assigns probablity into all cells of vector
 
}

std::vector<double> updateState(const std::vector<double>& previous_state, bool motion, bool observation)
{
	// Declare a new state vector with the same size as previous_state
    // All values in state are initialised with 0.

	std::vector<double> state(previous_state.size());									//current state sized to previous state vector
	std::vector<double> predicted(previous_state.size());								//predicted prob : motion updated vector
	std::vector<double> unnormalised(previous_state.size());							//unnormalised prob : observation updated
	
	double sum = 0;																		//variable to hold sum of all probablities

	initialiseState(predicted);															//initialise the motion update vector
	
	
    // Motion update - predicted probablity
	// Loop that sums all values in previous state vector
  	for (auto it = previous_state.begin() ; it != previous_state.end() ; it++)
	{
		sum += *it;
	}
	
	cout << sum  << endl ;																//prints sum for testing purposes : should be equal to 1 on first test 

	//loop that iteratively updates motion update in the  belief state					
	for (int k = 0 ; k < (cells+1) ; k ++)												//current state : k and previous state : i
	{			
		for (int i = 0 ; i < (cells+1) ; i ++)
		{
			predicted[k] += previous_state[i] * motionProbability(motion , k , i );		//Mathematic expression to determine motion update
		}
	}
	

    // Observation update
	for (int k  = 0 ; k < (cells+1); k++ )
	{
		unnormalised[k] = predicted[k] * observationProbability(observation , k);		//Mathematic expression to determine obsveration update
	}

	state = unnormalised;																//updates the state with unnormalised version
	
	normaliseState(state);																// Normalise the state vector
  
  return state;
}

void printState(const std::vector<double>& state)
{
  cout << "Position:    ";

  for (int i = 0; i < (state.size() -1); ++i)
  {
    cout << i << "     ";

    if (i < 10)
      cout << " ";
  }

  cout << (state.size() - 1) << endl;

  cout << "Probability: ";

  // Iterate through the state vector until the second last value
  for (auto it = state.begin(); it < (state.end() - 1); ++it)
  {
    cout << *it << " ";
  }

  // Print the last value and a new line
  cout << state.back() << endl;
}

int main()
{
  // Use fixed precision with 1 digit after the decimal
  cout.precision(1);
  cout << std::fixed;

  cout << endl;

  // Test the motionProbability function
  cout << "Testing motionProbability function:" << endl;

  cout << "Probability given no motion input, previous position 13 and current position 13:"
       << "\t" << motionProbability(false, 13, 13) << endl;

  cout << "Probability given no motion input, previous position 4 and current position 5:"
       << "\t\t" << motionProbability(false, 4, 5) << endl;

  cout << "Probability given no motion input, previous position 6 and current position 3:"
       << "\t\t" << motionProbability(false, 6, 3) << endl;

  cout << "Probability given a motion input, previous position 1 and current position 1:"
       << "\t\t" << motionProbability(true, 1, 1) << endl;

  cout << "Probability given a motion input, previous position 8 and current position 9:"
       << "\t\t" << motionProbability(true, 8, 9) << endl;

  cout << "Probability given a motion input, previous position 11 and current position 13:"
       << "\t\t" << motionProbability(true, 11, 13) << endl;

  cout << "Probability given a motion input, previous position 17 and current position 16:"
       << "\t\t" << motionProbability(true, 17, 16) << endl;

  cout << "Probability given a motion input, previous position 19 and current position 0:"
       << "\t\t" << motionProbability(true, 19, 0) << endl;

  cout << "Probability given a motion input, previous position 19 and current position 1:"
       << "\t\t" << motionProbability(true, 19, 1) << endl;

  cout << "Probability given a motion input, previous position 18 and current position 0:"
       << "\t\t" << motionProbability(true, 18, 0) << endl;

  cout << "Probability given a motion input, previous position 18 and current position 1:"
       << "\t\t" << motionProbability(true, 18, 1) << endl;

  cout << endl;

  // Test the observationProbability function
  cout << "Testing observationProbability function:" << endl;

  cout << "Probability given no observation input and position 0:" << "\t" << observationProbability(false, 0) << endl;
  cout << "Probability given no observation input and position 7:" << "\t" << observationProbability(false, 7) << endl;
  cout << "Probability given no observation input and position 13:" << "\t" << observationProbability(false, 13) << endl;

  cout << "Probability given an observation input and position 0:" << "\t" << observationProbability(true, 0) << endl;
  cout << "Probability given an observation input and position 7:" << "\t" << observationProbability(true, 7) << endl;
  cout << "Probability given an observation input and position 13:" << "\t" << observationProbability(true, 13) << endl;

  cout << endl;

  // Use fixed precision with 4 digits after the decimal
  cout.precision(4);
  cout << std::fixed;

  // Test normalisation
  cout << "Testing normaliseState function:" << endl;

  std::vector<double> test = {0.5, 1., 0.5, 2., 1.};

  cout << "Vector before normalisation:" << endl;
  printState(test);

  normaliseState(test);

  cout << "Vector after normalisation:" << endl;
  printState(test);

  // Declare the state vector
  std::vector<double> state;

  //put value of number of doors
  state.resize(cells);

  cout << endl;

  // Print the state before initialisation
  cout << "Before initialistion:" << endl;
  printState(state);
  cout << endl;

  // Initialise the state
  initialiseState(state);

  // Print the state after initialisation
  cout << "After initialistion:" << endl;
  printState(state);
  cout << endl;

  // First update
  cout << "Step 1 (no motion, door observed)" << endl;
  state = updateState(state, false, true);
  printState(state);
  cout << endl;


  // Second update
  cout << "Step 2 (motion, no door observed)" << endl;
  state = updateState(state, true, false);
  printState(state);
  cout << endl;


  // Third update
  cout << "Step 3 (motion, door observed)" << endl;
  state = updateState(state, true, true);
  printState(state);
  cout << endl;

  // Fourth update
  cout << "Step 4 (motion, no door observed)" << endl;
  state = updateState(state, true, false);
  printState(state);
  cout << endl;


  // Fifth update
  cout << "Step 5 (motion, no door observed)" << endl;
  state = updateState(state, true, false);
  printState(state);
  cout << endl;


  return 0;

}
