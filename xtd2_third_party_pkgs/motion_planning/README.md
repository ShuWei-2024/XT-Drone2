# Astra planner selection

The upstream EGO-Planner gitlink (`413a3f73` at upstream commit `15b2ef6`)
had no entry in `.gitmodules`, so recursive submodule checkout failed.
The Astra branch replaces that unused gitlink with this note and builds the
validated ROS 2 Jazzy planner from `astra/packages/ego_planner_jazzy`.
Its readiness and trajectory protections remain part of the Astra integration.
