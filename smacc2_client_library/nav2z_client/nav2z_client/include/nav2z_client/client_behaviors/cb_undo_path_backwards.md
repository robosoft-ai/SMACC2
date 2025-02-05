== DEVELOPER NOTES == 

- The cb_undo_path_backwards has a compatibility problem with nav2 in the way goal checkers are handled.
- When we have previously used the cb_backwards motion in a previous state, this selects the "BackwardsController" and the "backwards_goal_checker". Then, when we use cb_undo_path_backwards we also use the same controller and for somee reason we do not undeerstand the switch of th goal checker to "undo_path_backards_goal_checker" is like ignored.
- Thee workaround we found is duplicating the controller "BackwardsController" -> "UndoBackwardsController", and for some reason, now when the cb_undo_path_backwards uses "UndoBackardsController" (statd explicitly in the creation), the goal checker change does work as expected.

We noticed about this in feb 2025 when we were developing sm_nav_test_7