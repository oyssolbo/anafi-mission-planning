(define (domain sar)
  (:requirements :typing :durative-actions :negative-preconditions)
  (:types 
    vehicle location locatable - object
    drone airplane helicopter - vehicle
    searcharea loiterarea helipad - location 
    person lv bat - locatable
  ) 
  (:predicates 
    (at ?veh - vehicle ?loc - location)
    (available ?loc - location)
    (has ?veh - vehicle ?obj - locatable)
    (searched ?area - searcharea)
    (isinarea ?loc - locatable ?area - location)
    (path ?locfrom - location ?locto - location)
    (flying ?veh - vehicle)
  )

  (:durative-action MOVE-DRONE
      :parameters (
        ?drone - drone 
        ?locfrom - location 
        ?locto - location 
        ; ?bat - bat
      )
      :duration (= ?duration 5)
      :condition (and 
          (at start (and 
            ; (has ?drone ?bat) ; Battery should be linear or changing with time 
            (at ?drone ?locfrom)
            (available ?locto)
            ; (flying ?drone)
            (path ?locfrom ?locto)
          ))
          ; (over all (and 
          ;   ; (available ?locto)
          ;   ; (has ?drone ?bat)
          ;   ; (flying ?drone)
          ; ))
      )
      :effect (and 
          ; (at start (and 
          ;   (not (at ?drone ?locfrom))
          ;   (available ?locfrom) ; Technically incorrect to assume available at once, as the drone spends some time to get out of an area
          ; ))
          (at end (and 
            (at ?drone ?locto)
            (not (available ?locto))
            (available ?locfrom)
          ))
      )
  )

  (:durative-action TAKEOFF
      :parameters (
        ?drone - drone 
        ?bat - bat 
        ?helipad - helipad
      )
      :duration (= ?duration 2)
      :condition (and 
          (at start (and 
            (at ?drone ?helipad)
            ; (not (flying ?drone))
            ; (has ?drone ?bat)
          ))
      )
      :effect (and 
          (at end (and 
            (flying ?drone)
          ))
      )
  )

  ; (:durative-action LAND
  ;     :parameters (?drone - drone ?helipad - helipad ?bat - bat)
  ;     :duration (= ?duration 2)
  ;     :condition (and 
  ;         (at start (and 
  ;           (at ?drone ?helipad)
  ;           (flying ?drone)
  ;         ))
  ;     )
  ;     :effect (and 
  ;         (at end (and
  ;           (not (flying ?drone)) 
  ;           (not (has ?drone ?bat)) ; Assuming that the battery becomes empty when landing the drone
  ;         ))
  ;     )
  ; )

  ; (:durative-action SEARCH
  ;     :parameters (?loc - searcharea ?bat - bat ?drone - drone)
  ;     :duration (= ?duration 60)
  ;     :condition (and 
  ;         (at start (and 
  ;           (available ?loc)
  ;           (has ?drone ?bat)
  ;           (not (searched ?loc))
  ;         ))
  ;         (over all (and 
  ;           (has ?drone ?bat) 
  ;           (available ?loc) ; In case that another vehicle enters the search area 
  ;           (not (searched ?loc)) ; not searched in case new information about which areas are searched are received
  ;         ))
  ;     )
  ;     :effect (and 
  ;         (at end (and
  ;           (searched ?loc) 
  ;         ))
  ;     )
  ; )
)




; May be useful for having a helicopter interrupt the drone after a person is located
; (:event event_name
;     :parameters ()
;     :precondition (and
;         trigger condition
;     )
;     :effect (and
;         discrete effect(s)
;     )
; )



  ; (:action MOVE
  ;   :parameters (
  ;     ?drone - drone 
  ;     ?locfrom - location 
  ;     ?locto - location 
  ;     ?bat - bat
  ;   )
  ;   :precondition (and  
  ;     (has ?drone ?bat) ; Battery should be linear or changing with time 
  ;     (at ?drone ?locfrom)
  ;     (available ?locto)
  ;     (flying ?drone)
  ;     (path ?locfrom ?locto)
  ;   )
  ;   :effect (and 
  ;     (not (at ?drone ?locfrom))
  ;     (available ?locfrom) ; Technically incorrect to assume available at once, as the drone spends some time to get out of an area 
  ;     (at ?drone ?locto)
  ;     (not (available ?locto))  
  ;   )
  ; )

  ; (:action TAKEOFF
  ;   :parameters (
  ;     ?drone - drone 
  ;     ?bat - bat 
  ;     ?helipad - helipad
  ;   )
  ;   :precondition (and 
  ;     (at ?drone ?helipad)
  ;     (not (flying ?drone))
  ;     (has ?drone ?bat)
  ;   )
  ;   :effect (and 
  ;     (flying ?drone)
  ;   )
  ; )
  

  ; (:action LAND
  ;   :parameters (
  ;     ?drone - drone
  ;     ?helipad - helipad 
  ;     ?bat - bat
  ;   )
  ;   :precondition (and 
  ;     (at ?drone ?helipad)
  ;     (flying ?drone)
  ;   )
  ;   :effect (and 
  ;     (not (flying ?drone))
  ;     (not (has ?drone ?bat))
  ;   )
  ; )
  
  ; (:action SEARCH
  ;     :parameters (
  ;       ?loc - searcharea 
  ;       ?bat - bat 
  ;       ?drone - drone
  ;     )
  ;     :precondition (and 
  ;       (available ?loc)
  ;       (has ?drone ?bat)
  ;       (not (searched ?loc))
  ;     )
  ;     :effect (and )
  ; )
  
  

