/** 交互绑定：按钮、键盘、地图选点。具体动作由 main.js 注入。 */
const Control = (() => {
  const KEY_MAP = {
    ArrowUp: "up", ArrowDown: "down", ArrowLeft: "left", ArrowRight: "right",
    w: "up", s: "down", a: "left", d: "right",
  };

  function bind(handlers) {
    document.querySelectorAll("[data-direction]").forEach((button) => {
      button.addEventListener("click", () => handlers.move(button.dataset.direction));
    });
    document.querySelector("#patrol-start").addEventListener("click", handlers.patrolStart);
    document.querySelector("#patrol-stop").addEventListener("click", handlers.patrolStop);
    document.querySelector("#reset").addEventListener("click", handlers.reset);
    document.querySelector("#reload").addEventListener("click", handlers.reload);

    document.querySelector("#map").addEventListener("click", (event) => {
      const cell = event.target.closest(".cell");
      if (cell) handlers.pickTarget(Number(cell.dataset.x), Number(cell.dataset.y));
    });

    document.addEventListener("keydown", (event) => {
      const direction = KEY_MAP[event.key];
      if (direction) {
        event.preventDefault();
        handlers.move(direction);
      }
    });
  }

  return { bind };
})();
