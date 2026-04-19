import React from "react";
import { AbsoluteFill, Audio, Sequence, staticFile } from "remotion";
import { Intro } from "./components/Intro";
import { CarClip } from "./components/CarClip";
import { Outro } from "./components/Outro";
import { CAR_CLIPS, FPS, INTRO_DURATION, OUTRO_DURATION } from "./lib/constants";

export const CarCollectionVideo: React.FC = () => {
  return (
    <AbsoluteFill style={{ backgroundColor: "#000" }}>
      {/* Audio: place Tshwala Bam (or any amapiano track) at public/audio/background.mp3 */}
      <Audio src={staticFile("audio/background.mp3")} />

      <Sequence from={0} durationInFrames={INTRO_DURATION}>
        <Intro />
      </Sequence>

      {CAR_CLIPS.map((clip, i) => {
        const startFrame =
          INTRO_DURATION +
          CAR_CLIPS.slice(0, i).reduce(
            (acc, c) => acc + Math.round(c.durationSecs * FPS),
            0
          );
        const duration = Math.round(clip.durationSecs * FPS);
        return (
          <Sequence key={clip.filename} from={startFrame} durationInFrames={duration}>
            <CarClip clip={clip} index={i} totalClips={CAR_CLIPS.length} />
          </Sequence>
        );
      })}

      <Sequence
        from={
          INTRO_DURATION +
          CAR_CLIPS.reduce((acc, c) => acc + Math.round(c.durationSecs * FPS), 0)
        }
        durationInFrames={OUTRO_DURATION}
      >
        <Outro />
      </Sequence>
    </AbsoluteFill>
  );
};
