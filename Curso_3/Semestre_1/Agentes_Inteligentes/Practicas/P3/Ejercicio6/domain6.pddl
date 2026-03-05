(define (domain domain6)
  (:requirements :strips :typing :equality :durative-actions)

  (:types
      robot box waypoint orientation - object
  )

  (:predicates
      (at ?r - robot ?wp - waypoint)
      (robot_has_orientation ?r - robot ?o - orientation)
      (connected ?from - waypoint ?to - waypoint ?o - orientation)
      (box_at ?b - box ?wp - waypoint)
      (is_empty ?wp - waypoint)
  )

  ; No necesitamos funciones de coste/batería para este ejercicio básico de tiempo

  (:durative-action move
      :parameters (?r - robot ?from ?to - waypoint ?o - orientation)
      :duration (= ?duration 10) ; Tarda 10 unidades de tiempo
      :condition (and
          (at start (at ?r ?from))
          (at start (robot_has_orientation ?r ?o))
          (over all (connected ?from ?to ?o))
          (at start (is_empty ?to)) ; Reservamos el destino al inicio
      )
      :effect (and
          (at start (not (at ?r ?from)))
          (at end (at ?r ?to))
          (at start (is_empty ?from)) ; Liberamos el origen al inicio
          (at start (not (is_empty ?to))) ; Ocupamos el destino al inicio (para evitar choques)
      )
  )

  (:durative-action turn
      :parameters (?r - robot ?from_o ?to_o - orientation)
      :duration (= ?duration 2) ; Girar es rápido
      :condition (and
          (at start (robot_has_orientation ?r ?from_o))
          (at start (not (= ?from_o ?to_o)))
      )
      :effect (and
          (at start (not (robot_has_orientation ?r ?from_o)))
          (at end (robot_has_orientation ?r ?to_o))
      )
  )

  (:durative-action push
      :parameters (?r - robot ?b - box ?r_from ?b_from ?b_to - waypoint ?o - orientation)
      :duration (= ?duration 15) ; Empujar es lento
      :condition (and
          (at start (at ?r ?r_from))
          (at start (box_at ?b ?b_from))
          (at start (is_empty ?b_to))
          (at start (robot_has_orientation ?r ?o))
          (over all (connected ?r_from ?b_from ?o))
          (over all (connected ?b_from ?b_to ?o))
      )
      :effect (and
          (at start (not (at ?r ?r_from)))
          (at end (at ?r ?b_from))
          (at start (not (box_at ?b ?b_from)))
          (at end (box_at ?b ?b_to))
          (at start (is_empty ?r_from))
          (at start (not (is_empty ?b_to))) ; Reservamos destino caja
          ; Nota: b_from queda ocupado por el robot al final
      )
  )
)
