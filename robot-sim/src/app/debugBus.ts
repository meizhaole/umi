export type DebugEvent = {
  time: number;
  type: string;
  detail: unknown;
};

type BrowserDebugBus = {
  events: DebugEvent[];
  publish: (type: string, detail: unknown) => void;
  clear: () => void;
};

declare global {
  interface Window {
    __ROBOT_SIM_DEBUG__?: BrowserDebugBus;
  }
}

const MAX_EVENTS = 200;

export const publishDebugEvent = (type: string, detail: unknown): void => {
  if (typeof window === 'undefined') return;

  const bus = window.__ROBOT_SIM_DEBUG__ ?? {
    events: [],
    publish: publishDebugEvent,
    clear: () => {
      window.__ROBOT_SIM_DEBUG__?.events.splice(0);
    },
  };
  const event = { time: Date.now(), type, detail };
  bus.events.push(event);
  if (bus.events.length > MAX_EVENTS) bus.events.shift();
  window.__ROBOT_SIM_DEBUG__ = bus;
  window.dispatchEvent(new CustomEvent('robot-sim:debug', { detail: event }));
};
