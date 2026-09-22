from run_benchmark import run_mission

res = run_mission(mode='guided')
print('Final State:', res['terminal_state'], 'Final Miss:', res['final_miss'])

for idx, (t, s) in enumerate(zip(res['logs']['time'], res['logs']['state'])):
    if s == 'INHIBITED_DUD':
        print(f"Inhibited at T={t:.2f}s")
        print(f"  Confidence: {res['logs']['confidence'][idx]:.1f}%")
        print(f"  NIS: {res['logs']['nis'][idx]:.2f}")
        print(f"  ZEM Miss: {res['logs']['zem_miss'][idx]:.1f}m")
        print(f"  True Pos: [{res['logs']['true_x'][idx]:.1f}, {res['logs']['true_y'][idx]:.1f}, {res['logs']['true_z'][idx]:.1f}]")
        break
