(define (problem numeric_fuel_p0)
  (:domain numeric_fuel)
  (:objects 
    drone_0 - drone
    ;a1 a2 a3 - location
    a1 a2 a3 a4 a5 - search_area
    l1 l2 - loiter_area
    h1 h2 - helipad
    ; bat - battery
    mar - marker
    p - person
    ; takeoff land move drop search hover - action
  )
  (:init 
    (at drone_0 h1)
    (has_landed drone_0)
    ; (has drone_0 bat)
    ; (has drone_0 mar)
    
    (is_not_searching drone_0)

    (= (velocity drone_0) 5 )
    (= (battery_level drone_0) 50 ) ; Start at little to no fuel, such that it should refuel at start
                                    ; May want to also test this using some higher fuel-levels, and thus get 
                                    ; some information how the algorithm performs with respect to different
                                    ; fuel levels

                                    ; Interesting observation that the planner sees that it must return to land
                                    ; around halfways, when it starts with around 50 % of fuel. But it would have
                                    ; been more efficient for the drone to refuel at the start of the mission

    (not_searched a1)
    (not_searched a2)
    (not_searched a3)
    (not_searched a4)
    (not_searched a5)

    ; (is_person p a3)

    (available a1)
    (available a2)
    (available a3)
    (available a4)
    (available a5)
    (available l1)
    (available l2)
    (available h2)

    (path a1 a2)
    (path a2 a1)
    (path a1 a3)
    (path a3 a1)
    (path a2 a3)
    (path a3 a2)
    (path a1 a4)
    (path a4 a1)    
    (path a2 a5)
    (path a5 a2)

    (path l1 a1)
    (path a1 l1)
    (path l1 a2)
    (path a2 l1)

    (path l2 a1)
    (path a1 l2)
    (path l2 a2)
    (path a2 l2)

    (path l1 h1)
    (path h1 l1)
    (path l2 h2)
    (path h2 l2)

    (= (distance h1 l1)   5   )
    (= (distance l1 h1)   5   )
    (= (distance l1 a1)   100 )
    (= (distance a1 l1)   100 )
    (= (distance a1 a2)   50  )
    (= (distance a2 a1)   50  )
    (= (distance a1 a3)   50  )
    (= (distance a3 a1)   50  )
    (= (distance a3 a2)   150 )
    (= (distance a2 a3)   150 )
    (= (distance h2 l2)   25  )
    (= (distance l2 h2)   25  )
    (= (distance l2 a2)   75  )
    (= (distance a2 l2)   75  )
    (= (distance a5 a2)   75  )
    (= (distance a2 a5)   75  )    
    (= (distance a1 a4)   75  )
    (= (distance a4 a1)   75  )

    

    ; (= (fuel_level) 0)

    ; ; Unsure whether this is actually required in our case
    ; (= (min_power_lvl_to_start_action takeoff) 25)
    ; (= (min_power_lvl_to_start_action land) 0)
    ; (= (min_power_lvl_to_start_action move) 20)
    ; (= (min_power_lvl_to_start_action search) 30)
    ; (= (min_power_lvl_to_start_action drop) 0)


    ; ; Estimated in the simulator
    ; ; These estimates will be a bit conservative for landing, takeoff and drop
    ; (= (battery_usage_per_time_unit takeoff) 0.06201)
    ; (= (battery_usage_per_time_unit move) 0.06201)
    ; (= (battery_usage_per_time_unit search) 0.06201)

    ; (= (battery_usage_per_time_unit land) 0.0590)
    ; (= (battery_usage_per_time_unit hover) 0.0590)
    ; (= (battery_usage_per_time_unit drop) 0.0590)


  )
  (:goal
    (and 
      (at drone_0 h1) 
      (searched a1)
      (searched a2)
      (searched a3)
      (searched a4)
      (searched a5)
      ; ; (preference (searched a1))
      ; ; (preference (searched a2))
      ; ; (preference (searched a3))
      (has_landed drone_0)
    )
  )
  ; TP-1 Solves this problem in 27.91 ms
  ; which is far more efficient than the similar problem solved by the
  ; Graphplan algorithm developed by Hinostroza 
)