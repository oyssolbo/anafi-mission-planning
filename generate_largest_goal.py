# Examample of algorithm while I was bored on the airport
import copy
import itertools
import datetime

def get_all_valid_goals(
    num_goals       : int,
    valid_subgoals  : list,
    debug           : bool  = False
  ) -> list:
  if debug:
    start_time = datetime.datetime.now()

  goals = []
  for depth in range(1, num_goals + 1):
    l = list(itertools.combinations(valid_subgoals, depth))
    for tup in l:
      if not tup:
        continue
      goals.append([*tup])

  if debug:
    duration = datetime.datetime.now() - start_time
    print("Duration for creating goals with", num_goals, "subgoals:", duration.microseconds, "us")

  return goals 


def sort_goals(goals : list, debug : bool = False) -> list:
  if debug:
    start_time = datetime.datetime.now()

  goals = [sorted(x) for x in goals]
  goals = [tuple(sorted(x)) for x in goals]
  goals = list(set(goals))
  goals = [list(g) for g in goals]
  goals = sorted(goals, key=len, reverse=True)

  if debug:
    duration = datetime.datetime.now() - start_time
    print("Duration for sorting goals:", duration.microseconds, "us")

  return goals


def main() -> None:
  num_goals : int = 20
  subgoals : list = ["g_" + str(i) for i in range(num_goals)]
  valid_subgoals : list = copy.copy(subgoals[:-2])
  
  goals = get_all_valid_goals(num_goals, valid_subgoals, debug=True)
  goals = sort_goals(goals, debug=True)
  print(goals[0])
  print()

if __name__ == '__main__':
  main()
