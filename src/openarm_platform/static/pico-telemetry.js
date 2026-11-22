'use strict';

const picoTelemetry = (() => {
    const sides = ['left', 'right'];
    const rows = sides.map(side => Array.from(document.getElementById(`telemetry-${side}`).querySelectorAll('td')));
    const status = document.getElementById('telemetry-state');
    const record = document.getElementById('record-telemetry');
    const [recordZh, recordEn] = record.querySelectorAll('span');
    const recordLimit = 12000;
    let latest = null;
    let receivedAt = -Infinity;
    let source;
    let timer;
    let samples;
    let lastScene;

    const fixed = value => Number.isFinite(value) ? value.toFixed(1) : '—';

    function render(snapshot) {
        rows.forEach((cells, index) => {
            const arm = snapshot?.arms[sides[index]];
            const values = arm ? [
                ...arm.joint_deg.map(fixed), fixed(arm.jaw_opening_deg), fixed(arm.grip_force_n),
                arm.tcp_m.map(value => fixed(value * 100)).join(' / '),
            ] : [];
            cells.forEach((cell, col) => { cell.textContent = values[col] ?? '—'; });
        });
        record.disabled = !snapshot && !samples;
        status.textContent = samples
            ? `${samples.length} 条 / samples${samples.length === recordLimit ? ' · 已满 / full' : ''}`
            : snapshot ? '实时 · 5 Hz / Live' : '无实时数据 / No live data';
    }

    function saveCsv(buffer) {
        const names = ['timestamp_utc', 'sim_time_s', 'scene', ...sides.flatMap(side => [
            ...Array.from({length: 7}, (_, i) => `${side}_j${i + 1}_deg`),
            ...Array.from({length: 7}, (_, i) => `${side}_j${i + 1}_deg_s`),
            `${side}_jaw_deg`, `${side}_grip_force_n`, ...['x', 'y', 'z'].map(axis => `${side}_tcp_${axis}_m`),
        ])];
        const data = buffer.map(snapshot => [
            new Date(snapshot.timestamp_ms).toISOString(), snapshot.sim_time_s, snapshot.scene,
            ...sides.flatMap(side => {
                const arm = snapshot.arms[side];
                return [...arm.joint_deg, ...arm.joint_velocity_deg_s,
                    arm.jaw_opening_deg, arm.grip_force_n, ...arm.tcp_m];
            }),
        ].join(','));
        const url = URL.createObjectURL(new Blob([[names.join(','), ...data].join('\r\n') + '\r\n'],
            {type: 'text/csv;charset=utf-8'}));
        const link = document.createElement('a');
        link.href = url;
        link.download = `openarm-sim-${new Date().toISOString().replaceAll(':', '-')}.csv`;
        link.click();
        setTimeout(() => URL.revokeObjectURL(url), 30000);
    }

    record.addEventListener('click', () => {
        if (samples) {
            if (samples.length) saveCsv(samples);
            samples = undefined;
        } else {
            samples = [];
        }
        recordZh.textContent = samples ? '结束并下载' : '开始采集';
        recordEn.textContent = samples ? 'Stop & download CSV' : 'Record CSV';
        record.setAttribute('aria-pressed', String(Boolean(samples)));
        render(latest);
    });

    async function poll(connection) {
        const started = performance.now();
        try {
            const response = await fetch('/simulation/pico/telemetry', {cache: 'no-store', signal: connection.signal});
            if (!response.ok) throw new Error('Telemetry unavailable');
            const snapshot = await response.json();
            if (source !== connection) return;
            latest = snapshot;
            receivedAt = performance.now();
            if (snapshot.scene !== lastScene) {
                lastScene = snapshot.scene;
                document.dispatchEvent(new CustomEvent('pico-scene', {detail: lastScene}));
            }
            if (samples && samples.length < recordLimit) samples.push(snapshot);
            render(latest);
        } catch {
            if (source === connection && performance.now() - receivedAt > 1250) {
                latest = null;
                render(null);
            }
        } finally {
            if (source === connection) {
                const delay = Math.max(0, 200 - (performance.now() - started));
                timer = setTimeout(() => poll(connection), delay);
            }
        }
    }

    return {
        current: () => performance.now() - receivedAt <= 1250 ? latest : null,
        setActive(active) {
            if (active && !source) {
                source = new AbortController();
                poll(source);
            } else if (!active && source) {
                clearTimeout(timer);
                source.abort();
                source = undefined;
                latest = null;
                render(null);
            }
        },
    };
})();
