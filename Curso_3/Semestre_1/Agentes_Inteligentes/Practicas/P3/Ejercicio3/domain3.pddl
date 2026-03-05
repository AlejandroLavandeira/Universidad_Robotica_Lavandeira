(define (domain domain3)
  (:requirements :strips :typing :equality)

  (:types
      robot box waypoint orientation - object
  )

  (:predicates
      ; Predicados existentes
      (at ?r - robot ?wp - waypoint)
      (robot_has_orientation ?r - robot ?o - orientation)
      (connected ?from - waypoint ?to - waypoint ?o - orientation)
      ; Nuevos predicados para cajas
      (box_at ?b - box ?wp - waypoint)
      (is_empty ?wp - waypoint)
  )

  ; Las acciones move y turn se mantienen sin cambios
  (:action move
      :parameters (?r - robot ?from ?to - waypoint ?o - orientation)
      :precondition (and
          (at ?r ?from)
          (robot_has_orientation ?r ?o)
          (connected ?from ?to ?o)
          (is_empty ?to) ; ¡Importante! El robot solo puede moverse a celdas vacías
      )
      :effect (and
          (not (at ?r ?from))
          (at ?r ?to)
          (is_empty ?from)
          (not (is_empty ?to))
      )
  )

  (:action turn
      :parameters (?r - robot ?from_o ?to_o - orientation)
      :precondition (and
          (robot_has_orientation ?r ?from_o)
          (not (= ?from_o ?to_o))
      )
      :effect (and
          (not (robot_has_orientation ?r ?from_o))
          (robot_has_orientation ?r ?to_o)
      )
  )

  ; --- NUEVA ACCIÓN DE EMPUJE ---
  (:action push
      :parameters (?r - robot ?b - box ?r_from ?b_from ?b_to - waypoint ?o - orientation)
      :precondition (and
          (at ?r ?r_from)
          (box_at ?b ?b_from)
          (is_empty ?b_to)
          (robot_has_orientation ?r ?o)
          (connected ?r_from ?b_from ?o)
          (connected ?b_from ?b_to ?o)
      )
      :effect (and
          (not (at ?r ?r_from))
          (at ?r ?b_from)
          (not (box_at ?b ?b_from))
          (box_at ?b ?b_to)
          (is_empty ?r_from)
          (not (is_empty ?b_to))
      )
  )
)
