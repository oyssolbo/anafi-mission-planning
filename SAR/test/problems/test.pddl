(define (problem test)
  (:domain sar)
  (:objects 
    ; D0 - drone
    D0 - vehicle 
    lv bat - locatable 
    ; H1 H2 - helipad
    ; A1 A2 A3 - searcharea
    H1 A1 - location
  )
  (:init 
    (at D0 H1)
    (available A1)
    ; (available A2)
    ; (available A3)
    ; (available H2)
    ; (path A1 A2) ; is it really necessary to define all of the combinations?
    ; (path A1 A3)
    ; (path A2 A3)
    ; (path A3 A2)
    ; (path A3 A1)
    (path A1 H1)
    (path H1 A1)
    ; (path H2 A2)
    ; (path A2 H2)
    ; (has D0 bat)
    ; (has D0 lv)
  )
  (:goal
    (and 
    ; not is not specified as a keyword / token
    ; https://github.com/aig-upf/temporal-planning/issues/7
      ; (not (flying D0)) 
      ; (at D0 H1)
      (flying D0) 

      ; (searched A1)
      ; (searched A2)
      ; (searched A3) 
      ; (not (has D0 bat))
    )
  )

  ; (:metric 
  ;   minimize (total-time)
  ; )
)