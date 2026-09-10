# Cyber Physical Robotics: Project

*Project description dated July 31, 2026. Original: [Project-description.pdf](Project-description.pdf).*

## 1. Overview

Train in the design of distributed and concurrent software, particularly as it
relates to robotic control, subject to communication challenges.

## 2. Implementation

A simulation in any programming language. A 2D grid (10x10) where a configurable
number of robots move according to rules. Targets show up randomly and persist
until captured by robots. Each robot has a unique ID.

### 2.1 Robot operation

At each turn, a robot can perform one action:

- Move forward one square, in the direction it is facing
- Turn left (rotate 90° counter-clockwise)
- Turn right (rotate 90° clockwise)
- Pick up target from its current position

At each turn, a robot can sense:

- If there is a target in the position directly in front
- If there are robots in the position directly in front (and their IDs)
- If there are robots in the same position it currently occupies (and their IDs)

At each turn, a robot can broadcast a message (all other robots receive it).
Format and payload of the message are up to the implementation.

### 2.2 Simulation behavior

```
while (number iterations < max-iterations)
    if (no more targets)
        generate random targets
    for all robots
        read input messages
    for all robots
        sense environment
        decide on action to perform
    Test for illegal behavior here
    for all robots
        perform action
    for all robots
        send output messages
```

Test for these illegal conditions:

- At least one robot attempts to pick up a target from a position where there is no target
- Exactly one robot attempts to pick up a target from a position where there is a target
- More than two robots attempt to pick up a target from a position where there is a target

In other words, targets can only be picked up validly by **exactly two robots, at
exactly the same time**.

### 2.3 Communication

Communication is subject to random delay following an exponential distribution.
For each message a robot sends, all other robots receive it at potentially
different times: the random delay is applied per receiver, not per message.

```
Procedure send-message(msg)
    for all robots
        r = generate-random-number ; exponential distribution
        input-message-queue <- append(msg, r)

Procedure read-messages()
    for all messages in input-message-queue
        if (r == 0)
            read msg
        else
            r = r - 1
```

## 3. Deliverables

Three milestones. At each, submit a report describing the implementation. No code
in the report: high-level descriptions only (flowcharts, state machines, CSPs,
Petri nets...) plus a presentation of the simulation.

- **Milestone 1**: No communication delay, no full robot behavior. Illustrate the
  failure cases: show the simulation detects all illegal conditions.
- **Milestone 2**: No communication delay. Full robot behavior: show that, given
  perfect communication, the distributed control works.
- **Milestone 3**: Communication delay as described above. Show the distributed
  control still works despite unreliable communication.
