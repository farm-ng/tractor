import uPlot from "uplot";

// A collection of hideous code, mostly copy-pasted from
// leeoniya.github.io/uPlot/demos
// and patched to compile in TS.

// column-highlights the hovered x index
export function columnHighlightPlugin({
  className,
  style = { backgroundColor: "rgba(51,204,255,0.3)" },
}: { className?: string; style?: { backgroundColor: string } } = {}) {
  let highlightEl: HTMLDivElement;
  let currIdx: number | undefined;

  function init(u: uPlot) {
    const underEl = u.root.querySelector(".u-under");
    const overEl = u.root.querySelector(".u-over");

    highlightEl = document.createElement("div");

    className && highlightEl.classList.add(className);

    uPlot.assign(highlightEl.style, {
      pointerEvents: "none",
      display: "none",
      position: "absolute",
      left: 0,
      top: 0,
      height: "100%",
      ...style,
    });

    underEl?.appendChild(highlightEl);

    // show/hide highlight on enter/exit
    overEl?.addEventListener("mouseenter", () => {
      (highlightEl.style.display as any) = null;
    });
    overEl?.addEventListener("mouseleave", () => {
      highlightEl.style.display = "none";
    });
  }

  function update(u: uPlot) {
    if (currIdx !== u.cursor.idx) {
      currIdx = u.cursor.idx;

      const dx = (u.scales.x.max || 1) - (u.scales.x.min || 0);
      const width = u.bbox.width / dx / devicePixelRatio;
      const left = u.valToPos(currIdx as number, "x") - width / 2;

      highlightEl.style.transform = "translateX(" + Math.round(left) + "px)";
      highlightEl.style.width = Math.round(width) + "px";
    }
  }

  return {
    opts: (_: uPlot, opts: uPlot.Options) => {
      uPlot.assign(opts, {
        cursor: {
          x: false,
          y: false,
        },
      });
    },
    hooks: {
      init: init,
      setCursor: update,
    },
  };
}

// converts the legend into a simple tooltip
export function legendAsTooltipPlugin({
  className,
  style = { backgroundColor: "rgba(255, 249, 196, 0.92)", color: "black" },
}: {
  className?: string;
  style?: { backgroundColor: string; color: string };
} = {}) {
  let legendEl: HTMLDivElement | null;

  function init(u: uPlot, _: uPlot.Options) {
    legendEl = u.root.querySelector(".u-legend");
    if (!legendEl) {
      console.warn("Could not find .u-legend");
      return;
    }

    legendEl.classList.remove("u-inline");
    className && legendEl.classList.add(className);

    uPlot.assign(legendEl.style, {
      textAlign: "left",
      pointerEvents: "none",
      display: "none",
      position: "absolute",
      left: 0,
      top: 0,
      zIndex: 100,
      boxShadow: "2px 2px 10px rgba(0,0,0,0.5)",
      ...style,
    });

    // hide series color markers
    const idents: NodeListOf<HTMLDivElement> = legendEl.querySelectorAll(
      ".u-marker"
    );

    for (let i = 0; i < idents.length; i++) idents[i].style.display = "none";

    const overEl: HTMLDivElement | null = u.root.querySelector(".u-over");

    if (!overEl) {
      console.warn("Could not find .u-over");
      return;
    }

    overEl.style.overflow = "visible";

    // move legend into plot bounds
    overEl.appendChild(legendEl);

    // show/hide tooltip on enter/exit
    overEl.addEventListener("mouseenter", () => {
      if (legendEl) {
        legendEl.style.display = "";
      }
    });
    overEl.addEventListener("mouseleave", () => {
      if (legendEl) {
        legendEl.style.display = "none";
      }
    });

    // let tooltip exit plot
    //	overEl.style.overflow = "visible";
  }

  function update(u: uPlot) {
    const { left, top } = u.cursor;

    if (!legendEl) {
      console.warn("Could not find .u-legend");
      return;
    }
    legendEl.style.transform = "translate(" + left + "px, " + top + "px)";
  }

  return {
    hooks: {
      init: init,
      setCursor: update,
    },
  };
}

export function boxesPlugin({
  gap = 2,
  shadowColor = "#000000",
  bodyMaxWidth = 20,
  shadowWidth = 2,
  bodyOutline = 1,
} = {}) {
  function drawBoxes(u: uPlot) {
    u.ctx.save();
    const offset = (shadowWidth % 2) / 2;
    u.ctx.translate(offset, offset);

    if (u.scales.x.min === undefined || u.scales.x.max === undefined) {
      console.warn("Invalid u.scales.x: ", u.scales.x);
      return;
    }

    for (let i = u.scales.x.min; i <= (u.scales.x.max || 0); i++) {
      let med = u.data[1][i];
      let q1 = u.data[2][i];
      let q3 = u.data[3][i];
      let min = u.data[4][i];
      let max = u.data[5][i];

      if (
        min == null ||
        max == null ||
        q1 == null ||
        q3 == null ||
        med == null
      ) {
        console.warn("Invalid u.data: ", u.data);
        return;
      }

      let timeAsX = u.valToPos(i, "x", true);
      let lowAsY = u.valToPos(min, "y", true);
      let highAsY = u.valToPos(max, "y", true);
      let openAsY = u.valToPos(q1, "y", true);
      let closeAsY = u.valToPos(q3, "y", true);
      let medAsY = u.valToPos(med, "y", true);

      // shadow rect
      let shadowHeight = Math.max(highAsY, lowAsY) - Math.min(highAsY, lowAsY);
      let shadowX = timeAsX;
      let shadowY = Math.min(highAsY, lowAsY);

      u.ctx.beginPath();
      u.ctx.setLineDash([4, 4]);
      u.ctx.lineWidth = shadowWidth;
      u.ctx.strokeStyle = shadowColor;
      u.ctx.moveTo(Math.round(shadowX), Math.round(shadowY));
      u.ctx.lineTo(Math.round(shadowX), Math.round(shadowY + shadowHeight));
      u.ctx.stroke();

      // body rect
      let columnWidth = u.bbox.width / (u.scales.x.max - u.scales.x.min);
      let bodyWidth = Math.min(bodyMaxWidth, columnWidth - gap);
      let bodyHeight =
        Math.max(closeAsY, openAsY) - Math.min(closeAsY, openAsY);
      let bodyX = timeAsX - bodyWidth / 2;
      let bodyY = Math.min(closeAsY, openAsY);
      let bodyColor = "#eee";

      u.ctx.fillStyle = shadowColor;
      u.ctx.fillRect(
        Math.round(bodyX),
        Math.round(bodyY),
        Math.round(bodyWidth),
        Math.round(bodyHeight)
      );

      u.ctx.fillStyle = bodyColor;
      u.ctx.fillRect(
        Math.round(bodyX + bodyOutline),
        Math.round(bodyY + bodyOutline),
        Math.round(bodyWidth - bodyOutline * 2),
        Math.round(bodyHeight - bodyOutline * 2)
      );

      u.ctx.fillStyle = "#000";
      u.ctx.fillRect(
        Math.round(bodyX),
        Math.round(medAsY - 1),
        Math.round(bodyWidth),
        Math.round(2)
      );

      // hz min/max whiskers
      u.ctx.beginPath();
      u.ctx.setLineDash([]);
      u.ctx.lineWidth = shadowWidth;
      u.ctx.strokeStyle = shadowColor;
      u.ctx.moveTo(Math.round(bodyX), Math.round(highAsY));
      u.ctx.lineTo(Math.round(bodyX + bodyWidth), Math.round(highAsY));
      u.ctx.moveTo(Math.round(bodyX), Math.round(lowAsY));
      u.ctx.lineTo(Math.round(bodyX + bodyWidth), Math.round(lowAsY));
      u.ctx.stroke();
    }
    u.ctx.translate(-offset, -offset);
    u.ctx.restore();
  }
  return {
    opts: (_: uPlot, opts: uPlot.Options) => {
      uPlot.assign(opts, {
        cursor: {
          points: {
            show: false,
          },
        },
      });

      opts.series.forEach((series) => {
        series.paths = undefined;
        series.points = { show: false };
      });
    },
    hooks: {
      draw: drawBoxes,
    },
  };
}
