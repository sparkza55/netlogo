turtles-own
[
  vx                ;; x velocity
  vy                ;; y velocity
  desired-direction ;; my desired direction
  driving-forcex    ;; my main motivating force
  driving-forcey
  obstacle-forcex   ;; force exerted by obstacles
  obstacle-forcey
  territorial-forcex;; force exerted by neighbors
  territorial-forcey
  status
  vi
  vf
  vm
  count-me-in
]

globals
[
  total-times       ;; a list containing 10 accumulatfors for total time spent waiting by agents with the nth decile pushiness value
  total-counts
  vt       ;; a list containing 10 accumulatfors for total time spent waiting by agents with the nth decile pushiness value
  d      ;; a list containing 10 accumulators or total times agents with the nth decile of pushiness was served
  xc1
  xc2
]

;; Set up the view
to setup
  ca
  import-pcolors "map1.jpg"
  set-default-shape turtles "circle"
  ;; initialize the globals
  set xc1 18
  set xc2 20
  set total-times (list 0 0 0 0 0 0 0 0 0 0)
  set total-counts (list 0 0 0 0 0 0 0 0 0 0)
  ;; create patrons
  ask patches with [((pxcor >= 10 and pxcor <= 11) and (pycor >= -1 and pycor <= 0))]
  [ set pcolor gray ]
  ask patches with [((pxcor >= xc1 and pxcor <= xc2) and (pycor >= -2 and pycor <= 2))]
  [ set pcolor yellow ]
  create-turtles random 3
  [
    setxy 0 (-6 + random 12)
    set count-me-in false
    ;; give the turtles an initial nudge towards the goal
    let init-direction 30 + random 90
    set vx sin init-direction
    set vy cos init-direction
    set status random 2
    color-turtle
  ]
  reset-ticks
end

;; run the simulation
to go
  if ticks mod 10 = 0
  [
    create-turtles patrons
    [
      setxy 0 (-6 + random 12)
      set count-me-in false
      ;; give the turtles an initial nudge towards the goal
      let init-direction 30 + random 90
      set vx sin init-direction
      set vy cos init-direction
      set status random 2
      color-turtle
    ]
  ]

  ask turtles with [xcor > 39]
  [ die ]

  ;; run the social forces model on thirsty turtles
  ;; calculate the forces first...
  ask turtles
  [
    calc-desired-direction
    calc-driving-force
    calc-obstacle-force
    if any? other turtles
      [ calc-territorial-forces ]
  ]
  ;; then move the turtles and have them grow impatient if need be
  ask turtles
  [
    move-turtle
    color-turtle
  ]

  ask turtles with [(xcor >= xc1 and xcor <= (xc1 + 1))][
    set count-me-in true
    set vi magnitude vx vy
    print vi
  ]

  ask turtles with [(xcor >= xc2 and xcor <= (xc2 + 1))][
    set vf magnitude vx vy
    set vm mean (list vi vf)

    set vt mean [vm] of turtles with [count-me-in = true]
    set d count turtles with [count-me-in = true]
  ]

  ask turtles with [xcor >= (xc2 + 1)] [
    set count-me-in false
  ]

  tick
end


;; color a turtle according to its pushiness
to color-turtle
  set color 19 - (patrons) * 4
end

;; helper function to find the magnitude of a vector
to-report magnitude [x y]
  report sqrt ((x ^ 2) + (y ^ 2))
end

;; returns 1 if the angle between the desired vector and the force vector is within a threshold, else return c
to-report field-of-view-modifier [desiredx desiredy forcex forcey]
  ifelse (desiredx * (- forcex) + desiredy * (- forcey)) >= (magnitude forcex forcey) * cos (field-of-view / 2)
  [ report 1 ]
  [ report c]
end

;;;; Functions for calculating the social forces ;;;;
;; move the turtle according to the rules of the social forces model
to move-turtle
  let ax driving-forcex + obstacle-forcex + territorial-forcex
  let ay driving-forcey + obstacle-forcey + territorial-forcey

  set vx vx + ax
  set vy vy + ay

  ;; scale down the velocity if it is too high
  let vmag magnitude vx vy
  let multiplier 1
  if vmag > max-speed
  [set multiplier max-speed / vmag]

  set vx vx * multiplier
  set vy vy * multiplier

  ifelse status = 0
  [
    set xcor xcor + vx
    set ycor ycor + vy
  ] [
    set xcor xcor + (vx / 2)
    set ycor ycor + (vy / 2)
  ]
end

;; find the territorial force according to the social forces model
to calc-territorial-forces
  set territorial-forcex 0
  set territorial-forcey 0
  ask other turtles with [distance myself > 0]
  [
    let to-agent (towards myself) - 180
    let rabx [xcor] of myself - xcor
    let raby [ycor] of myself - ycor
    let speed magnitude vx vy
    let to-root ((magnitude rabx raby) + (magnitude (rabx - (speed * sin desired-direction)) (raby - (speed * cos desired-direction)))) ^ 2 - speed ^ 2
    if to-root < 0
    [set to-root 0]
    let b 0.5 * sqrt to-root

    let agent-force (- v0) * exp (- b / sigma)

    ask myself
    [
      let agent-forcex agent-force * (sin to-agent)
      let agent-forcey agent-force * (cos to-agent)
      ;; modify the effect this force has based on whether or not it is in the field of view
      let vision field-of-view-modifier driving-forcex driving-forcey agent-forcex agent-forcey
      set territorial-forcex territorial-forcex + agent-forcex * vision
      set territorial-forcey territorial-forcey + agent-forcey * vision
    ]
  ]
end

;; find the obstacle force of the turtle according to the social forces model
to calc-obstacle-force
  set obstacle-forcex 0
  set obstacle-forcey 0
  ask patches with [pcolor = 14.9]
  [
    let to-obstacle (towards myself) - 180
    let obstacle-force (- u0) * exp (- (distance myself) / r)
    ask myself
    [
     set obstacle-forcex obstacle-forcex + obstacle-force * (sin to-obstacle)
     set obstacle-forcey obstacle-forcey + obstacle-force * (cos to-obstacle)
    ]
  ]
end

;; find the driving force of the turtle
to calc-driving-force
  set driving-forcex (1 / tau) * (max-speed * (sin desired-direction) - vx)
  set driving-forcey (1 / tau) * (max-speed * (cos desired-direction) - vy)
end

;; find the heading towards the nearest goal
to calc-desired-direction
  let goal min-one-of (patches with [pxcor = 39])
  [ distance myself ]
  set desired-direction towards goal
end
