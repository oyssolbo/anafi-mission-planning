(define (domain search_and_rescue)
  (:requirements 
    :strips 
    :typing 
    :adl 
    :fluents 
    :durative-actions
  )

  ;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  (:types
    drone 
    ship
    location
    person
    marker
    lifevest
    action
    trackable ; helipad or person 
  );; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

  ;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
  (:predicates
    (drone_at ?d - drone ?loc - location)
    (person_at ?p - person ?loc - location)
    (path ?loc_from - location ?loc_to - location)
    
    ; Resupply with respect to both battery and equipment
    (can_recharge ?loc - location)
    (can_resupply ?loc - location)

    ; Duplication of predicates, since POPF cannot handle negative preconditions...
    (landed ?d - drone)
    (not_landed ?d - drone)

    (searched ?loc - location)
    (not_searched ?d - drone)
    
    ; Perhaps some of these are not necessary, but better safe than sorry...
    (not_searching ?d - drone)
    (not_rescuing ?d - drone)
    (not_marking ?d - drone)

    (tracking ?d - drone ?loc - location) ; Tracking either the helipad or a person to be rescued
    (not_tracking ?d - drone ?loc - location) 

    ; marked and rescued should not be necessary to enforce, as these should be set by
    ; the planner, using preferences in soft-goals. For example, prefer rescuing but 
    ; disallow it when the severity is too low
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
    (search_velocity ?d - drone) 
    
    (severity ?p - person ?loc - location) ; Severity levels: 0, 1, 2 (for now) with 2 being the most critical
    
    (battery_charge ?d - drone)
    (battery_usage ?d - drone)
    
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
        (over all(not_tracking ?d ?loc_from))
        (over all(not_marking ?d))
      )
      :effect (and
        (at start(not(drone_at ?d ?loc_from))) ; If an error occurs during execution, the mission controller must be advanced enough
        (at end(drone_at ?d ?loc_to))
      )
  )


  (:durative-action land
      :parameters (?d - drone ?loc - location)
      :duration ( = ?duration 10)
      :condition (and
        (at start(drone_at ?d ?loc))
        (at start(not_landed ?d))
        (over all(tracking ?d ?loc))
      )
      :effect (and
        (at end(and
          (landed ?d) 
          (not (not_landed ?d))
        ))
      )
  )


  (:durative-action takeoff
      :parameters (?d - drone ?loc - location)
      :duration ( = ?duration 5)
      :condition (and
          (at start(drone_at ?d ?loc))
          (at start(landed ?d))
      )
      :effect (and
          (at end(not(landed ?d)))
          (at end(not_landed ?d))
      )
  )


  (:durative-action search
      :parameters (?d - drone ?loc - location)
      :duration ( = ?duration (/ (search_distance ?loc) (search_velocity ?d)))
      :condition (and
        (at start(drone_at ?d ?loc))
        (at start(not_searching ?d))
        (over all(not_landed ?d))
      )
      :effect (and
        ; (over all(not(not_searching ?d))) ; Why tf does this not work?? Fuck PDDL
        (at start(not(not_searching ?d)))
        (at end(not_searching ?d))
        (at end(searched ?loc))
      )
  )


  (:durative-action track
      :parameters (?d - drone ?loc - location)
      :duration (= ?duration 20)
      :condition (and 
          (at start(searched ?loc))
          (over all (drone_at ?d ?loc))
      )
      :effect (and 
          (at start (tracking ?d ?loc))
          (at start (not(not_tracking ?d ?loc)))
          (at end (not(tracking ?d ?loc)))
          (at end (not_tracking ?d ?loc))
      )
  )


  (:durative-action drop_marker
      :parameters (?d - drone ?loc - location ?p - person ?m - marker)
      :duration ( = ?duration 2)
      :condition (and
        (at start(drone_at ?d ?loc))
        (at start(person_at ?p ?loc))
        (at start(not_marked ?p ?loc))
        (at start(tracking ?d ?loc))
        (at start(>=(severity ?p ?loc) 1)) 
        (at start(>=(num_markers ?d) 1))
        (over all(not_landed ?d))
      )
      :effect (and
        (at start(not(not_marking ?d)))
        (at end(not_marking ?d))
        (at end(decrease (num_markers ?d) 1))
        (at end(marked ?p ?loc))
      )
  )


  (:durative-action drop_lifevest
      :parameters (?d - drone ?loc - location ?p - person ?l - lifevest)
      :duration ( = ?duration 2)
      :condition (and
        (at start(drone_at ?d ?loc))
        (at start(person_at ?p ?loc))
        (at start(not_rescued ?p ?loc))
        (at start(tracking ?d ?loc))
        (at start(>=(severity ?p ?loc) 2)) 
        (at start(>=(num_lifevests ?d) 1))
        (over all(not_landed ?d))
      )
      :effect (and
        (at start(not(not_rescuing ?d)))
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
      :duration ( = ?duration (+ (- 1 (num_lifevests ?d)) (- 2 (num_markers ?d)))) ; Time dependent on adding markers and lifevests
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
