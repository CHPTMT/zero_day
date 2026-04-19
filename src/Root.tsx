import React from "react";
import { Composition } from "remotion";
import { CarCollectionVideo } from "./CarCollectionVideo";
import { FPS, WIDTH, HEIGHT, TOTAL_FRAMES } from "./lib/constants";

export const Root: React.FC = () => {
  return (
    <Composition
      id="CarCollection"
      component={CarCollectionVideo}
      durationInFrames={TOTAL_FRAMES}
      fps={FPS}
      width={WIDTH}
      height={HEIGHT}
    />
  );
};
