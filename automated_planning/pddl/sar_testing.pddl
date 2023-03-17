(define (domain search_and_rescue)
  (:requirements 
    :strips 
    :equality 
    :typing 
    :fluents 
    :durative-actions 
    ;:numeric-fluents
    ; :duration-inequalities
    ; :disjunctive-preconditions ; Not supported by PlanSys2 POPF, but standard POPF does not fail with this...
  )

  ;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  (:types
    drone ship - vehicle
    marker lifevest - equipment
    person location - trackable 
  );; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

  ;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
  (:predicates
    (drone_at ?d - drone ?loc - location)
    (person_at ?p - person ?loc - location)
    (path ?loc_from - location ?loc_to - location)
    
    ; Resupply with respect to both battery and equipment
    (can_recharge ?loc - location)
    (can_resupply ?loc - location)
    (can_land ?loc - location)

    ; Duplication of predicates, since POPF cannot handle negative preconditions...
    (landed ?d - drone)
    (not_landed ?d - drone)

    (searched ?loc - location)
    (not_searched ?loc - location)
    
    ; Perhaps some of these are not necessary, but better safe than sorry...
    (not_searching ?d - drone)
    (not_rescuing ?d - drone)
    (not_marking ?d - drone)
    (not_tracking ?d - drone)
    (not_moving ?d - drone)

    (tracked ?p - person) ; Tracking either a person or a landing location
    (not_tracked ?p - person) ; Theory that type trackable not working

    (marked ?p - person ?loc - location) ; Dropping marker
    (not_marked ?p - person ?loc - location)

    (rescued ?p - person ?loc - location) ; Dropping lifevest
    (not_rescued ?p - person ?loc - location)

    (communicated ?p - person ?loc - location) ; Communicating about position
    (not_communicated ?p - person ?loc - location)

  );; end Predicates ;;;;;;;;;;;;;;;;;;;;
  
  ;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
  (:functions
    (distance ?loc_from - location ?loc_to - location)
    (search_distance ?loc - location)

    (move_velocity ?d - drone)
    (track_velocity ?d - drone) 
    
    (battery_charge ?d - drone)
    (move_battery_usage ?d - drone)
    (track_battery_usage ?d - drone)
    
    (num_markers ?d - drone)
    (num_lifevests ?d - drone)    
  );; end Functions ;;;;;;;;;;;;;;;;;;;;
  
  ;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  (:durative-action move
      :parameters (?d - drone ?loc_from - location ?loc_to - location)
      :duration ( = ?duration (/ (distance ?loc_from ?loc_to) (move_velocity ?d)))
      :condition (and
        (at start(path ?loc_from ?loc_to))
        (at start(drone_at ?d ?loc_from))
        (over all(not_landed ?d))
        (over all(not_searching ?d))
        (over all(not_rescuing ?d))
        (over all(not_marking ?d))
        (over all(not_tracking ?d))
      )
      :effect (and
        ; (decrease (battery_charge ?d) (* (move_battery_usage ?d) #t))
        (at start (decrease (battery_charge ?d) (* (move_battery_usage ?d) (/ (distance ?loc_from ?loc_to) (move_velocity ?d))))) ; Using at-start to prevent issues with concurrent actions
        ; (at end (decrease (battery_charge ?d) 5)) ; Only fixed values are supported...

        (at start(not (drone_at ?d ?loc_from)))
        (at start(not (not_moving ?d)))
        (at end(not_moving ?d))
        (at end(drone_at ?d ?loc_to))
      )
  )


  ; The landing sequence becomes utterly wrong by removing the (tracked ?loc)
  ; Need to find a method for tracking the helipad / location beforehand
  (:durative-action land
      :parameters (?d - drone ?loc - location); ?helipad - person)
      :duration ( = ?duration 10)
      :condition (and
        (at start (>= (battery_charge ?d) 15)) ; Force it to land with some battery remaining 
        (at start(drone_at ?d ?loc))
        (at start(not_landed ?d))
        ; (at start(tracked ?helipad)) ; Method for circumwenting the problem by classifying the helipad as a 'person' 
        (at start(searched ?loc))
        (over all(can_land ?loc)) ; Possible to add go-nogo states for the landing locations in the future
        (over all(not_moving ?d)) ; Prevent the drone from moving to a different location during a landing 
      )
      :effect (and
        (at end (decrease (battery_charge ?d) 15)) 
        (at end(landed ?d))
        (at end(not (not_landed ?d)))
      )
  )


  (:durative-action takeoff
      :parameters (?d - drone ?loc - location); ?helipad - person)
      :duration ( = ?duration 5)
      :condition (and
        (at start (>= (battery_charge ?d) 25)) ; Add this in the config file
        (at start(drone_at ?d ?loc))
        (at start(landed ?d))
      )
      :effect (and
        (at end (decrease (battery_charge ?d) 2)) 
        (at end(not (landed ?d)))
        (at end(not_landed ?d))
        (at end(not_searched ?loc))  ; Landing pad can have moved until next landing. Setting this forces a search
        (at end(not (searched ?loc)))
        ; (at end(not (tracked ?helipad)))  ; and retract of the helipad
        ; (at end(not_searched ?loc))
        ; (at end(not_tracked ?helipad))
      )
  )


  (:durative-action search
      :parameters (?d - drone ?loc - location)
      :duration ( = ?duration (/ (search_distance ?loc) (track_velocity ?d)))
      :condition (and
        (at start (> (battery_charge ?d) (* (track_battery_usage ?d) (/ (search_distance ?loc) (track_velocity ?d))))) ; Add this using the config file
        (at start(drone_at ?d ?loc))
        (at start(not_searching ?d))
        (at start(not_searched ?loc))
        (over all(not_landed ?d))
        (over all(not_moving ?d))
        (at start (>= (num_markers ?d) 1))
      )
      :effect (and
        ; (at end (decrease (battery_charge ?d) 1))
        ; (decrease (battery_charge ?d) 10));
        (at start (decrease (battery_charge ?d) (* (track_battery_usage ?d) (/ (search_distance ?loc) (track_velocity ?d))))) ; Using at-start to prevent issues with concurrent actions
        ; (decrease (battery_charge ?d) (* (track_battery_usage ?d) #t))

        ; (over all(not (not_searching ?d))) ; Why tf does this not work?? Fuck PDDL
        (at start(not (not_searching ?d)))
        (at end(not_searching ?d))
        (at end(searched ?loc))
        (at end (decrease (num_markers ?d) 1))
      )
  )


  (:durative-action track
      :parameters (?d - drone ?loc - location ?p - person)
      :duration (= ?duration 20)
      :condition (and 
        (at start(searched ?loc))
        (at start (not_tracked ?p))
        (at start (person_at ?p ?loc))
        (over all (drone_at ?d ?loc))
        (over all(not_moving ?d))
      )
      :effect (and
        (at start(not (not_tracking ?d)))
        (at end(not_tracking ?d))
        (at end (tracked ?p))
        (at end (not (not_tracked ?p))) 
      )
  )


  (:durative-action drop_marker
      :parameters (?d - drone ?loc - location ?p - person ?m - marker)
      :duration ( = ?duration 2)
      :condition (and
        (at start(drone_at ?d ?loc))
        (at start(person_at ?p ?loc))
        (at start(not_marked ?p ?loc))
        (at start(tracked ?p))
        (at start(>= (num_markers ?d) 1))
        (over all(not_landed ?d))
        (over all(not_moving ?d))
      )
      :effect (and
        (at start (decrease (battery_charge ?d) (* (track_battery_usage ?d) 2)))
        (at start(not (not_marking ?d)))
        (at end(not_marking ?d))
        (at end(decrease (num_markers ?d) 1))
        (at end(marked ?p ?loc))
      )
  )


  (:durative-action communicate
      :parameters (?d - drone ?loc - location ?p - person)
      :duration ( = ?duration 1)
      :condition (and
        (at start(drone_at ?d ?loc))
        (at start(person_at ?p ?loc))
        (at start(not_communicated ?p ?loc))
        (at start(tracked ?p))
        (over all(not_landed ?d))
        (over all(not_moving ?d))
      )
      :effect (and
        (at end(not (not_communicated ?p ?loc)))
        (at end(communicated ?p ?loc))
      )
  )


  (:durative-action drop_lifevest
      :parameters (?d - drone ?loc - location ?p - person ?l - lifevest)
      :duration ( = ?duration 2)
      :condition (and
        (at start(drone_at ?d ?loc))
        (at start(person_at ?p ?loc))
        (at start(not_rescued ?p ?loc))
        ; (at start(tracked ?p))
        (at start(>= (num_lifevests ?d) 1))
        (over all(not_landed ?d))
        (over all(not_moving ?d))
      )
      :effect (and
        (at start (decrease (battery_charge ?d) (* (track_battery_usage ?d) 2)))
        (at start(not (not_rescuing ?d)))
        (at end(not_rescuing ?d))
        (at end(decrease (num_lifevests ?d) 1))
        (at end(rescued ?p ?loc))
      )
  )


  (:durative-action recharge
      :parameters (?d - drone ?loc - location)
      :duration ( = ?duration (* 0.25 (- 100 (battery_charge ?d)))) ; 4 percent per time unit
      :condition (and
        (at start(drone_at ?d ?loc))
        (at start(can_recharge ?loc))
        (at start(< (battery_charge ?d) 100))
        (over all(landed ?d))
      )
      :effect (and
        (at end(assign (battery_charge ?d) 100)) ; Could extend this to increasing the charge with respect to time
      )
  )


  (:durative-action resupply
      :parameters (?d - drone ?loc - location)
      :duration ( = ?duration (+ (* 10 (- 1 (num_lifevests ?d))) (* 5 (- 2 (num_markers ?d))))) ; Time dependent on adding markers and lifevests
      :condition (and
        (at start(drone_at ?d ?loc))
        (at start(can_resupply ?loc))
        (at start(<= (num_lifevests ?d) 1))
        (at start(<= (num_markers ?d) 2))
        (over all(landed ?d))
      )
      :effect (and
        (at end(assign (num_lifevests ?d) 1))
        (at end(assign (num_markers ?d) 2))
      )
  )


);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
