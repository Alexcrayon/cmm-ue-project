# Corridor Map Method for Pathfinding in Unreal Engine 5

A UE5 project implementing the Corridor Map Method (CMM) as an alternative pathfinding approach to grid-based A*. Developed for CS5150 Game AI at Northeastern University, Spring 2026.

Part of the codebase is based on course assignments (grid system, A* pathfinding, perception system).

## Requirements

- Unreal Engine 5.7
- Visual Studio 2022

## Setup

1. Clone the repository
2. Right-click the `.uproject` file and select "Generate Visual Studio project files"
3. Open the generated `.sln` in Visual Studio
4. Build and run the project

## Running the Project

The project defaults to `TestLevel_cmm`. The `TestLevelAssignment5` map is also usable.

### CMM Setup

1. In the level, select the `CMMDataActor`
2. In the Details panel, assign the `GridActor` property to the `GridActor` present in the level
3. Play the level — CMM data (distance transform, skeleton, backbone graph) is built automatically on startup
4. The AI agent will use the backbone graph for pathfinding

### Debug Visualization

Inside the `BP_CMMDataActor`, multiple debug visualization function can be called.
default to backbone graph visualization.



Note: only enable one debug visualization at a time, as they share the same debug texture on the grid actor.

## Project Structure

- `Source/GameAI/CMM/` — CMM implementation
  - `GACMMDataActor` — offline construction of distance transform, skeleton, and backbone graph
  - `GACMMPathComponent` — A* pathfinding on the backbone graph and path following


## References

- Geraerts, R. & Overmars, M. (2007). "The Corridor Map Method: Real-Time High-Quality Path Planning."
- Geraerts, R. et al. (2008). "Using the CMM for Path Planning for a Large Number of Characters."
- Geraerts, R. & Overmars, M. (2008). "Enhancing Corridor Maps for Real-Time Path Planning in Virtual Environments."
- Karlsson, T. (2026). "The Corridor Map Method." In Game AI Uncovered: Volume Four.
