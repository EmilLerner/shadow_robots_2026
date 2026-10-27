"""Run live dual-arm prediction and simulated UR5 IK/FK comparisons."""
import time
from DualLivePrediction import XsensCollector, LivePrediction

UPDATE_RATE = 80
RADIO_CHANNEL = 19
BUFFER_LIMIT = 32
REPORT_INTERVAL = 5.0
PRINT_EVERY = 20


def main():
    collector = XsensCollector(UPDATE_RATE, RADIO_CHANNEL)
    predictor = LivePrediction(buffer_limit=BUFFER_LIMIT)
    last_report = time.perf_counter()
    comparisons = 0
    print('Live Xsens prediction + UR5 kinematic evaluation')
    print('Right arm: NONE | Left arm: INV')
    print(f'Prediction horizon: {predictor.horizon * 1000:.1f} ms')
    print('No live resampling, CSV saving, or robot connection.')
    print('Robot positions are IK/FK references, NOT physical measurements.\n')
    try:
        collector.start()
        while True:
            row = collector.poll()
            if row is not None:
                for event in predictor.process(row):
                    if event[0] == 'human':
                        comparisons += 1
                        if comparisons % PRINT_EVERY == 0:
                            print(f'Human 3D errors (cm): {event[2].round(2)} '
                                  f'| target timing overshoot: {event[1]*1000:.1f} ms')
                    elif event[0] == 'ik_failed' and comparisons % PRINT_EVERY == 0:
                        print(f'IK warning: {event[1]}')
            now = time.perf_counter()
            if now - last_report >= REPORT_INTERVAL:
                print(f'\nComplete Xsens samples: {collector.sample_count}')
                print(predictor.summary(), '\n')
                last_report = now
            time.sleep(0.001)
    except KeyboardInterrupt:
        print('\nStopping...')
    finally:
        collector.stop()
        print('\nFINAL RESULTS')
        print(predictor.summary())
        print('Sensor packet counts:', collector.packet_counts)


if __name__ == '__main__':
    main()
