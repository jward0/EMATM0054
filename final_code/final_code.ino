enum state_enum {
  INITIALISATION,
  SEARCHING,
  FOLLOWING,
  LINE_END,
  CONTROLLED_ROTATING,
  CONTROLLED_DRIVING,
  FINISHED,
  S_ERROR
};

state_enum state;

void setup() {
  // put your setup code here, to run once:

}

void loop() {

  switch(state) {
    case INITIALISATION:
      break;

    case SEARCHING:
      break;

    case FOLLOWING:
      break;

    case LINE_END:
      break;

    case CONTROLLED_ROTATING:
      break;

    case CONTROLLED_DRIVING:
      break;

    case FINISHED:
      break;

    case S_ERROR:
      break;
  }

  update_state();

}

void update_state() {

  switch(state) {
    case INITIALISATION:
      break;

    case SEARCHING:
      break;

    case FOLLOWING:
      break;

    case LINE_END:
      break;

    case CONTROLLED_ROTATING:
      break;

    case CONTROLLED_DRIVING:
      break;

    case FINISHED:
      break;

    case S_ERROR:
      break;
  }
}
