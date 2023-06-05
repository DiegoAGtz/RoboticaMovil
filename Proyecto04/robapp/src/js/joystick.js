export class Joystick {
    constructor(id, maxDistance, deadZone) {
        this.initMove = (x, y) => {
            this.active = true;
            if (this.stick)
                this.stick.style.transition = "0s";
            this.dragStart = { x, y };
        };
        this.move = (x, y) => {
            const xDiff = x - this.dragStart.x;
            const yDiff = y - this.dragStart.y;
            const angle = Math.atan2(yDiff, xDiff);
            const distance = Math.min(this.maxDistance, Math.hypot(xDiff, yDiff));
            const xPosition = distance * Math.cos(angle);
            const yPosition = distance * Math.sin(angle);
            if (this.stick)
                this.stick.style.transform = `translate3d(${xPosition}px, ${yPosition}px, 0px)`;
            const distance2 = distance < this.deadZone
                ? 0
                : (this.maxDistance / (this.maxDistance - this.deadZone)) *
                    (distance - this.deadZone);
            const xPosition2 = distance2 * Math.cos(angle);
            const yPosition2 = distance2 * Math.sin(angle);
            const xPercent = parseFloat((xPosition2 / this.maxDistance).toFixed(4));
            const yPercent = parseFloat((yPosition2 / this.maxDistance).toFixed(4));
            this.value = { x: xPercent, y: yPercent };
        };
        this.endMove = () => {
            if (!this.active)
                return;
            if (this.stick) {
                this.stick.style.transition = ".2s";
                this.stick.style.transform = "translate3d(0px, 0px, 0px)";
            }
            this.value = { x: 0, y: 0 };
            this.touchId = -1;
            this.active = false;
        };
        this.touchStart = (event) => {
            event.preventDefault();
            this.initMove(event.changedTouches[0].clientX, event.changedTouches[0].clientY);
            this.touchId = event.changedTouches[0].identifier;
        };
        this.touchMove = (event) => {
            if (!this.active)
                return;
            let touchMoveId = false;
            let x = 0, y = 0;
            if (event.changedTouches) {
                for (let i = 0; i > event.changedTouches.length; i++) {
                    if (this.touchId == event.changedTouches[i].identifier) {
                        touchMoveId = true;
                        x = event.changedTouches[i].clientX;
                        y = event.changedTouches[i].clientY;
                    }
                }
                if (!touchMoveId)
                    return;
            }
            this.move(x, y);
        };
        this.touchUp = (event) => {
            if (event.changedTouches &&
                this.touchId != event.changedTouches[0].identifier)
                return;
            this.endMove();
        };
        this.mouseDown = (event) => {
            event.preventDefault();
            this.initMove(event.clientX, event.clientY);
        };
        this.mouseMove = (event) => {
            if (!this.active)
                return;
            this.move(event.clientX, event.clientY);
        };
        this.mouseUp = (_) => {
            this.endMove();
        };
        this.addListeners = () => {
            var _a, _b;
            (_a = this.stick) === null || _a === void 0 ? void 0 : _a.addEventListener("mousedown", this.mouseDown);
            (_b = this.stick) === null || _b === void 0 ? void 0 : _b.addEventListener("touchstart", this.touchStart);
            document.addEventListener("mousemove", this.mouseMove, {
                passive: false,
            });
            document.addEventListener("touchmove", this.touchMove, {
                passive: false,
            });
            document.addEventListener("mouseup", this.mouseUp);
            document.addEventListener("touchend", this.touchUp);
        };
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
}
