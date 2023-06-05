export class Joystick {
  id: string;
  stick: HTMLElement | null;
  active: boolean;
  maxDistance: number;
  deadZone: number;
  dragStart: { x: number; y: number };
  touchId: number;
  value: { x: number; y: number };

  constructor(id: string, maxDistance: number, deadZone: number) {
    this.id = id;
    this.stick = document.getElementById(id);
    this.active = false;
    this.maxDistance = maxDistance;
    this.deadZone = deadZone;
    this.dragStart = { x: 0, y: 0 };
    this.touchId = -1;
    this.value = { x: 0, y: 0 };
    this.addListeners();
  }

  initMove = (x: number, y: number): void => {
    this.active = true;
    if (this.stick) this.stick.style.transition = "0s";
    this.dragStart = { x, y };
  }

  move = (x: number, y: number): void => {
    const xDiff = x - this.dragStart.x;
    const yDiff = y - this.dragStart.y;
    const angle = Math.atan2(yDiff, xDiff);
    const distance = Math.min(this.maxDistance, Math.hypot(xDiff, yDiff));
    const xPosition = distance * Math.cos(angle);
    const yPosition = distance * Math.sin(angle);

    if (this.stick)
      this.stick.style.transform = `translate3d(${xPosition}px, ${yPosition}px, 0px)`;

    const distance2 =
      distance < this.deadZone
        ? 0
        : (this.maxDistance / (this.maxDistance - this.deadZone)) *
          (distance - this.deadZone);
    const xPosition2 = distance2 * Math.cos(angle);
    const yPosition2 = distance2 * Math.sin(angle);
    const xPercent = parseFloat((xPosition2 / this.maxDistance).toFixed(4));
    const yPercent = parseFloat((yPosition2 / this.maxDistance).toFixed(4));

    this.value = { x: xPercent, y: yPercent };
  }

  endMove = (): void => {
    if (!this.active) return;
    if (this.stick) {
      this.stick.style.transition = ".2s";
      this.stick.style.transform = "translate3d(0px, 0px, 0px)";
    }
    this.value = { x: 0, y: 0 };
    this.touchId = -1;
    this.active = false;
  }

  touchStart = (event: TouchEvent): void => {
    event.preventDefault();
    this.initMove(
      event.changedTouches[0].clientX,
      event.changedTouches[0].clientY
    );
    this.touchId = event.changedTouches[0].identifier;
  }

  touchMove = (event: TouchEvent): void => {
    if (!this.active) return;
    let touchMoveId = false;
    let x = 0,
      y = 0;
    if (event.changedTouches) {
      for (let i = 0; i > event.changedTouches.length; i++) {
        if (this.touchId == event.changedTouches[i].identifier) {
          touchMoveId = true;
          x = event.changedTouches[i].clientX;
          y = event.changedTouches[i].clientY;
        }
      }
      if (!touchMoveId) return;
    }
    this.move(x, y);
  }

  touchUp = (event: TouchEvent): void => {
    if (
      event.changedTouches &&
      this.touchId != event.changedTouches[0].identifier
    )
      return;
    this.endMove();
  }

  mouseDown = (event: MouseEvent): void => {
    event.preventDefault();
    this.initMove(event.clientX, event.clientY);
  }

  mouseMove = (event: MouseEvent): void => {
    if (!this.active) return;
    this.move(event.clientX, event.clientY);
  }

  mouseUp = (_: MouseEvent): void => {
    this.endMove();
  }

  addListeners = (): void => {
    this.stick?.addEventListener("mousedown", this.mouseDown);
    this.stick?.addEventListener("touchstart", this.touchStart);
    document.addEventListener("mousemove", this.mouseMove, {
      passive: false,
    });
    document.addEventListener("touchmove", this.touchMove, {
      passive: false,
    });
    document.addEventListener("mouseup", this.mouseUp);
    document.addEventListener("touchend", this.touchUp);
  }
}