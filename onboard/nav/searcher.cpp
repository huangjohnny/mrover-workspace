#include "searcher.hpp"
#include "utilities.hpp"
#include "stateMachine.hpp"

#include <iostream>

Searcher::Searcher(StateMachine* stateMachine_)
  : /*currentState(SearchState::SearchFaceNorth)
  ,*/ stateMachine(stateMachine_) { }


NavState Searcher::run()
{
    // if( !stateMachine->mPhoebe->roverStatus().autonState().is_auton )
    // {
    //     // currentState = SearchState::SearchFaceNorth;
    //     return NavState::Off;
    // }
	switch ( stateMachine->mPhoebe->roverStatus().currentState() )
	{
	    case NavState::SearchFaceNorth:
	    {
	      return executeSearchFaceNorth();
	    }

	    case NavState::SearchFace120:
	    {
	      return executeSearchFace120();
	    }

	    case NavState::SearchFace240:
	    {
	      return executeSearchFace240();
	    }

	    case NavState::SearchFace360:
	    {
	      return executeSearchFace360();
	    }

	    case NavState::SearchTurn:
	    {
	      return executeSearchTurn();
	    }

	    case NavState::SearchDrive:
	    {
	      return executeSearchDrive();
	    }

	    case NavState::TurnToBall:
	    {
	      return executeTurnToBall();
	    }

	    case NavState::DriveToBall:
	    {
	      return executeDriveToBall();
	    }
        default:
        {
            return NavState::Unknown;
        }
	} // switch
}

// Executes the logic for turning to face north to orient itself for
// a search. If the rover is turned off, it proceeds to Off. If the
// rover detects the tennis ball, it proceeds to the ball If the rover
// finishes turning, it proceeds to SearchFace120. Else the rover keeps
// turning to north.
NavState Searcher::executeSearchFaceNorth()
{
	// if( !stateMachine->mPhoebe->roverStatus().autonState().is_auton )
	// {
 //    	currentState = SearchState::SearchFaceNorth;
	// 	return NavState::Off;
	// }
	if( stateMachine->mPhoebe->roverStatus().tennisBall().found )
	{
    	// currentState = SearchState::TurnToBall;
		return NavState::TurnToBall;
	}
	if( stateMachine->mPhoebe->turn( 90 ) )
	{
    	// currentState = SearchState::SearchFace120;
		return NavState::SearchFace120;
	}

  	// currentState = SearchState::SearchFaceNorth;
	return NavState::SearchFaceNorth;
} // executeSearchFaceNorth

// Executes the logic for the first third of the initial 360 degree
// turn of the search. If the rover is turned off, the rover proceeds
// to Off. If the rover detects the tennis ball, it proceeds to the
// ball. If the rover finishes turning, it proceeds to SearchFace240.
// Else the rover keeps turning to 120 degrees.
NavState Searcher::executeSearchFace120()
{
	// if( !stateMachine->mPhoebe->roverStatus().autonState().is_auton )
	// {
 //    	currentState = SearchState::SearchFaceNorth;
	// 	return NavState::Off;
	// }
	if( stateMachine->mPhoebe->roverStatus().tennisBall().found )
	{
    	// currentState = SearchState::TurnToBall;
		return NavState::TurnToBall;
	}
	if( stateMachine->mPhoebe->turn( 210 ) )
	{
    	// currentState = SearchState::SearchFace240;
		return NavState::SearchFace240;
	}

    // currentState = SearchState::SearchFace120;
    return NavState::SearchFace120;
} // executeSearchFace120()

// Executes the logic for the second third of the initial 360 degree
// turn of the search. If the rover is turned off, the rover proceeds
// to Off. If the rover detects the tennis ball, it proceeds to the
// ball. If the rover finishes turning, it proceeds to SearchFace360.
// Else the rover keeps turning to 240 degrees.
NavState Searcher::executeSearchFace240()
{
	// if( !stateMachine->mPhoebe->roverStatus().autonState().is_auton )
	// {
 //    currentState = SearchState::SearchFaceNorth;
	// 	return NavState::Off;
	// }
	if( stateMachine->mPhoebe->roverStatus().tennisBall().found )
	{
    // currentState = SearchState::TurnToBall;
    return NavState::TurnToBall;
	}
	if( stateMachine->mPhoebe->turn( 330 ) )
	{
    // currentState = SearchState::SearchFace360;
    return NavState::SearchFace360;
	}
  // currentState = SearchState::SearchFace240;
  return NavState::SearchFace240;
} // executeSearchFace240

// Executes the logic for the final third of the initial 360 degree
// turn of the search. If the rover is turned off, the rover proceeds
// to Off. If the rover detects the tennis ball, it proceeds to the
// ball. If the rover finishes turning, the next state is SearchDrive.
// Else the rover keeps turning to 360 degrees.
NavState Searcher::executeSearchFace360()
{
	// if( !stateMachine->mPhoebe->roverStatus().autonState().is_auton )
	// {
 //    currentState = SearchState::SearchFaceNorth;
	// 	return NavState::Off;
	// }
	if( stateMachine->mPhoebe->roverStatus().tennisBall().found )
	{
    // currentState = SearchState::TurnToBall;
    return NavState::TurnToBall;
	}
	if( stateMachine->mPhoebe->turn( 90 ) )
	{
		initializeSearch();
    // currentState = SearchState::SearchTurn;
    return NavState::SearchTurn;
	}
  // currentState = SearchState::SearchFace360;
  return NavState::SearchFace360;
} // executeSearchFace360()

// Executes the logic for turning while searching. If the rover is
// turned off, the rover proceeds to Off. If the rover detects the
// tennis ball, it proceeds to the ball. If the rover finishes turning,
// it proceeds to driving while searching. Else the rover keeps
// turning to the next Waypoint.
NavState Searcher::executeSearchTurn()
{
	// if( !stateMachine->mPhoebe->roverStatus().autonState().is_auton )
	// {
 //    	currentState = SearchState::SearchFaceNorth;
	// 	return NavState::Off;
	// }
	if( stateMachine->mPhoebe->roverStatus().tennisBall().found )
	{
    	// currentState = SearchState::TurnToBall;
    	return NavState::TurnToBall;
	}

	if( mSearchPoints.empty() )
	{
		if( !addFourPointsToSearch() )
		{
			stateMachine->mPhoebe->roverStatus().path().pop();
			stateMachine->mMissedWaypoints += 1;
      		// currentState = SearchState::SearchFaceNorth; // todo
			return NavState::Turn;
		}
	}

	Odometry& nextSearchPoint = mSearchPoints.front();
	if( stateMachine->mPhoebe->turn( nextSearchPoint ) )
	{
    	// currentState = SearchState::SearchDrive;
   		return NavState::SearchDrive;
	}

  // currentState = SearchState::SearchTurn;
  return NavState::SearchTurn;
} // executeSearchTurn()

// Executes the logic for driving while searching. If the rover is
// turned off, the rover proceeds to Off. If the rover detects the
// tennis ball, it proceeds to the ball. If the rover finishes driving,
// it proceeds to turning to the next Waypoint. If the rover detects
// an obstacle, it proceeds to obstacle avoidance. Else the rover
// keeps driving to the next Waypoint.
NavState Searcher::executeSearchDrive()
{
	// if( !stateMachine->mPhoebe->roverStatus().autonState().is_auton )
	// {
 //   		currentState = SearchState::SearchFaceNorth;
	// 	return NavState::Off;
	// }
	if( stateMachine->mPhoebe->roverStatus().tennisBall().found )
	{
    	// currentState = SearchState::TurnToBall;
    	return NavState::TurnToBall;
	}

	if( stateMachine->mPhoebe->roverStatus().obstacle().detected )
	{
        stateMachine->mOriginalObstacleAngle = stateMachine->mPhoebe->roverStatus().obstacle().bearing;
    	// currentState = SearchState::SearchTurn; // todo
		return NavState::SearchTurnAroundObs;
	}

	const Odometry& nextSearchPoint = mSearchPoints.front();
	DriveStatus driveStatus = stateMachine->mPhoebe->drive( nextSearchPoint );
	if( driveStatus == DriveStatus::Arrived )
	{
		mSearchPoints.pop();
    	// currentState = SearchState::SearchTurn;
		return NavState::SearchTurn;
	}
	if( driveStatus == DriveStatus::OnCourse )
	{
    	// currentState = SearchState::SearchDrive;
		return NavState::SearchDrive;
	}
	// if driveStatus == DriveStatus::OffCourse
  	// currentState = SearchState::SearchTurn;
  	return NavState::SearchTurn;
} // executeSearchDrive()

// Executes the logic for turning to the tennis ball. If the rover is
// turned off, it proceeds to Off. If the rover loses the ball, it
// starts to search again. If the rover finishes turning to the ball,
// it drives to the ball. Else the rover continues to turn to to the
// ball.
NavState Searcher::executeTurnToBall()
{
	// if( !stateMachine->mPhoebe->roverStatus().autonState().is_auton )
	// {
 //    	currentState = SearchState::SearchFaceNorth;
	// 	return NavState::Off;
	// }
	if( !stateMachine->mPhoebe->roverStatus().tennisBall().found )
	{
    	// currentState = SearchState::SearchFaceNorth;
		return NavState::SearchFaceNorth;
	}
	if( stateMachine->mPhoebe->turn( stateMachine->mPhoebe->roverStatus().tennisBall().bearing +
					stateMachine->mPhoebe->roverStatus().bearing().bearing ) )
	{
    	// currentState = SearchState::DriveToBall;
		return NavState::DriveToBall;
	}

    // currentState = SearchState::TurnToBall;
    return NavState::TurnToBall;
} // executeTurnToBall()

// Executes the logic for driving to the tennis ball. If the rover is
// turned off, it proceeds to Off. If the rover loses the ball, it
// starts the search again. If the rover detects an obstacle, it
// proceeds to go around the obstacle. If the rover finishes driving
// to the ball, it moves on to the next Waypoint. If the rover gets
// off course, it proceeds to turn back to the Waypoint. Else, it
// continues driving to the ball.
NavState Searcher::executeDriveToBall()
{
	// if( !stateMachine->mPhoebe->roverStatus().autonState().is_auton )
	// {
 //    	currentState = SearchState::SearchFaceNorth;
	// 	return NavState::Off;
	// }
	if( !stateMachine->mPhoebe->roverStatus().tennisBall().found )
	{
    	// currentState = SearchState::SearchFaceNorth;
		return NavState::SearchFaceNorth;
	}
	// TODO: save location of ball then go around object?
	if( stateMachine->mPhoebe->roverStatus().obstacle().detected )
	{
        stateMachine->mOriginalObstacleAngle = stateMachine->mPhoebe->roverStatus().obstacle().bearing;
    	// currentState = SearchState::SearchFaceNorth; // todo
		return NavState::SearchTurnAroundObs;
	}
	DriveStatus driveStatus = stateMachine->mPhoebe->drive( stateMachine->mPhoebe->roverStatus().tennisBall().distance,
											stateMachine->mPhoebe->roverStatus().tennisBall().bearing +
                                            stateMachine->mPhoebe->roverStatus().bearing().bearing );
	if( driveStatus == DriveStatus::Arrived )
	{
		stateMachine->mPhoebe->roverStatus().path().pop();
        stateMachine->mCompletedWaypoints += 1;
    	// currentState = SearchState::SearchFaceNorth; // todo
		return NavState::Turn;
	}
	if( driveStatus == DriveStatus::OnCourse )
	{
   		// currentState = SearchState::DriveToBall;
		return NavState::DriveToBall;
	}
	// if driveStatus == DriveStatus::OffCourse
  	// currentState = SearchState::TurnToBall;
  	return NavState::TurnToBall;
} // executeDriveToBall()

// Initializes the search ponit multipliers to be the intermost loop
// of the search.
void Searcher::initializeSearch()
{
	clear( mSearchPoints );
	mSearchPointMultipliers.clear();
	mSearchPointMultipliers.push_back( pair<short, short> ( 0, 1 ) );
	mSearchPointMultipliers.push_back( pair<short, short> ( -1, 1 ) );
	mSearchPointMultipliers.push_back( pair<short, short> ( -1, -1 ) );
	mSearchPointMultipliers.push_back( pair<short, short> ( 1, -1 ) );
	addFourPointsToSearch();
} // initializeSearch()

// true indicates to added search points
// Add the next loop to the search. If the points are added to the
// search, returns true. If the rover is further away from the start
// of the search than the search bail threshold, return false.
bool Searcher::addFourPointsToSearch()
{
	const double pathWidth = stateMachine->mRoverConfig[ "pathWidth" ].GetDouble();
	if( mSearchPointMultipliers[ 0 ].second * pathWidth > stateMachine->mRoverConfig[ "searchBailThresh" ].GetDouble() )
	{
		return false;
	}

	for( auto& mSearchPointMultiplier : mSearchPointMultipliers )
	{
		Odometry nextSearchPoint = stateMachine->mPhoebe->roverStatus().path().front().odom;
		double totalLatitudeMinutes = nextSearchPoint.latitude_min +
			( mSearchPointMultiplier.first * pathWidth  * LAT_METER_IN_MINUTES );
		double totalLongitudeMinutes = nextSearchPoint.longitude_min +
			( mSearchPointMultiplier.second * pathWidth * stateMachine->mPhoebe->longMeterInMinutes() );

		nextSearchPoint.latitude_deg += totalLatitudeMinutes / 60;
		nextSearchPoint.latitude_min = ( totalLatitudeMinutes - ( ( (int) totalLatitudeMinutes ) / 60 ) * 60 );
		nextSearchPoint.longitude_deg += totalLongitudeMinutes / 60;
		nextSearchPoint.longitude_min = ( totalLongitudeMinutes - ( ( (int) totalLongitudeMinutes) / 60 ) * 60 );

		mSearchPoints.push( nextSearchPoint );

		mSearchPointMultiplier.first < 0 ? --mSearchPointMultiplier.first : ++mSearchPointMultiplier.first;
		mSearchPointMultiplier.second < 0 ? --mSearchPointMultiplier.second : ++mSearchPointMultiplier.second;
	}
	return true;
} // addFourPointsToSearch()
