#ifndef SEARCHER_HPP
#define SEARCHER_HPP

#include "rover.hpp"

class StateMachine;

class Searcher {
public:
  Searcher(StateMachine* stateMachine_);

  NavState run();

  // Queue of search points.
  queue<Odometry> mSearchPoints;

private:
  enum class SearchState
  {
    SearchFaceNorth,
    SearchFace120,
    SearchFace240,
    SearchFace360,
    SearchTurn,
    SearchDrive,
    TurnToBall,
    DriveToBall
  };

  NavState executeSearchFaceNorth();

  NavState executeSearchFace120();

  NavState executeSearchFace240();

  NavState executeSearchFace360();

  NavState executeSearchTurn();

  NavState executeSearchDrive();

  NavState executeTurnToBall();

  NavState executeDriveToBall();

  void initializeSearch();

  bool addFourPointsToSearch();

  // Vector of search point multipliers used as a base for the search
  // points.
  vector< pair<short, short> > mSearchPointMultipliers;


public:

    // SearchState currentState;

    StateMachine* stateMachine;

};

#endif //SEARCHER_HPP
