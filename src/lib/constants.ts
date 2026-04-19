export const FPS = 30;
export const WIDTH = 1080;
export const HEIGHT = 1920;

// Tshwala Bam by TitoM & Yuppe ft. S.N.E & EeQue – ~114 BPM
export const BPM = 114;
export const FRAMES_PER_BEAT = Math.round((60 / BPM) * FPS);

export const INTRO_DURATION = 2 * FPS;   // 60 frames
export const OUTRO_DURATION = 3 * FPS;   // 90 frames

export interface CarClipConfig {
  filename: string;
  label: string;
  durationSecs: number;
}

// Drop your proxy files into public/videos/ and rename them car1.mp4 … car6.mp4
// (or update filenames here to match your actual proxy names)
export const CAR_CLIPS: CarClipConfig[] = [
  { filename: "car1.mp4", label: "THE BEAST", durationSecs: 5 },
  { filename: "car2.mp4", label: "PURE POWER", durationSecs: 5 },
  { filename: "car3.mp4", label: "STREET KING", durationSecs: 5 },
  { filename: "car4.mp4", label: "NIGHT RUNNER", durationSecs: 5 },
  { filename: "car5.mp4", label: "THE DAILY", durationSecs: 5 },
  { filename: "car6.mp4", label: "THE CLASSIC", durationSecs: 5 },
];

export const CLIPS_DURATION = CAR_CLIPS.reduce(
  (acc, c) => acc + Math.round(c.durationSecs * FPS),
  0
);

// Total: 60 + 900 + 90 = 1050 frames = 35 seconds
export const TOTAL_FRAMES = INTRO_DURATION + CLIPS_DURATION + OUTRO_DURATION;
