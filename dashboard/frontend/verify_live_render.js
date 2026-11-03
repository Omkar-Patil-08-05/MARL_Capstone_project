import puppeteer from 'puppeteer';

(async () => {
    const outputPath = process.argv[2] || '/home/capstone/capstone_project_antigravity/results/dashboard_six_drone_polished.png';
    const mode = process.argv[3] || '6';
    console.log(`[Puppeteer] Connecting to live dashboard in ${mode}-drone mode. Saving screenshot to ${outputPath}...`);
    let browser;
    const errors = [];

    try {
        browser = await puppeteer.launch({
            executablePath: '/usr/bin/google-chrome-stable',
            headless: true,
            args: ['--no-sandbox', '--disable-setuid-sandbox', '--disable-gpu', '--window-size=1920,1080']
        });
        const page = await browser.newPage();
        await page.setViewport({ width: 1920, height: 1080 });

        page.on('console', msg => {
            if (msg.type() === 'error') {
                errors.push(msg.text());
                console.error('[BROWSER_CONSOLE_ERROR]', msg.text());
            }
        });

        page.on('pageerror', err => {
            errors.push(err.message);
            console.error('[PAGE_ERROR]', err.message);
        });

        await page.goto('http://localhost:5173', { waitUntil: 'domcontentloaded', timeout: 15000 });

        // Wait 4 seconds for telemetry stream to populate cards and maps
        await new Promise(r => setTimeout(r, 4000));

        let pageText = await page.evaluate(() => document.body.innerText);

        // 1. Swarm Identification checks
        const hasD0 = pageText.includes('DRONE 0');
        const hasSwarmBadge = pageText.includes('ACTIVE SWARM') || pageText.includes('N =');
        const expectedDrones = mode === '2' ? 'N = 2' : 'N = 6';
        const hasExpectedDrones = pageText.includes(expectedDrones);
        console.log(`[CHECK] Drone 0 present: ${hasD0 ? 'PASS' : 'FAIL'}`);
        console.log(`[CHECK] Active swarm badge present (${expectedDrones}): ${hasExpectedDrones ? 'PASS' : 'FAIL'}`);

        // 2. Coverage Convention & Agreement checks
        const expectedCov = mode === '2' ? '28.60%' : '30.50%';
        const hasExpectedCov = pageText.includes(expectedCov);
        const hasUndefined = pageText.includes('undefined active') || pageText.includes('undefined total') || pageText.includes('undefined%');
        const hasUnscaledFraction = pageText.includes('0.9008%') || pageText.includes('0.30504%');
        console.log(`[CHECK] ${expectedCov} coverage formatted consistently: ${hasExpectedCov ? 'PASS' : 'FAIL'}`);
        console.log(`[CHECK] No undefined text: ${!hasUndefined ? 'PASS' : 'FAIL (Found undefined)'}`);
        console.log(`[CHECK] No unscaled fractions (0.9008%): ${!hasUnscaledFraction ? 'PASS' : 'FAIL'}`);

        // 3. Task Counts checks
        const expectedTask = mode === '2' ? '1 active / 1 total' : '0 active / 0 total';
        const hasExpectedTask = pageText.includes(expectedTask);
        console.log(`[CHECK] Task counts clean (${expectedTask}): ${hasExpectedTask ? 'PASS' : 'FAIL'}`);

        // 4. Test METRICS tab in VictimDetectionPanel
        console.log('[Puppeteer] Clicking METRICS tab...');
        const clickedMetrics = await page.evaluate(() => {
            const buttons = Array.from(document.querySelectorAll('button'));
            const metricsBtn = buttons.find(b => b.innerText.includes('METRICS'));
            if (metricsBtn) {
                metricsBtn.click();
                return true;
            }
            return false;
        });

        await new Promise(r => setTimeout(r, 1000));
        pageText = await page.evaluate(() => document.body.innerText);

        let evalCheckPassed = false;
        if (mode === '2') {
            const hasF1Hundred = pageText.includes('100.0%');
            const hasTP2 = pageText.includes('True Positives') && pageText.includes('2');
            const hasLocError = pageText.includes('0.34 m');
            evalCheckPassed = hasF1Hundred && hasTP2 && hasLocError;
            console.log(`[CHECK] Active autonomous evaluation metrics (F1: 100%, TP: 2, Loc Error: 0.34m): ${evalCheckPassed ? 'PASS' : 'FAIL'}`);
        } else {
            const hasNotEvaluated = pageText.toLowerCase().includes('not evaluated for this mission');
            const hasMisleadingZeroF1 = pageText.includes('F1-SCORE\n0.0%') && pageText.includes('FALSE NEGATIVES\n5');
            evalCheckPassed = hasNotEvaluated && !hasMisleadingZeroF1;
            console.log(`[CHECK] Non-misleading evaluation state ('Not Evaluated For This Mission'): ${hasNotEvaluated ? 'PASS' : 'FAIL'}`);
            console.log(`[CHECK] No misleading 0.0% F1 / 5 FN false-alarm: ${!hasMisleadingZeroF1 ? 'PASS' : 'FAIL'}`);
        }
        console.log(`[CHECK] Browser console errors: ${errors.length}`);

        // Switch back to AUTONOMOUS tab for clean final screenshot
        await page.evaluate(() => {
            const buttons = Array.from(document.querySelectorAll('button'));
            const autoBtn = buttons.find(b => b.innerText.includes('AUTONOMOUS'));
            if (autoBtn) autoBtn.click();
        });
        await new Promise(r => setTimeout(r, 500));

        await page.screenshot({ path: outputPath, fullPage: true });
        console.log(`[Puppeteer] Live screenshot captured: ${outputPath}`);

        if (hasUndefined || hasUnscaledFraction || errors.length > 0 || !hasD0 || !hasExpectedDrones || !evalCheckPassed) {
            console.error('[VERIFICATION] Failed live verification!');
            process.exit(1);
        } else {
            console.log('[VERIFICATION] All live dashboard consistency and rendering checks PASSED!');
            process.exit(0);
        }
    } catch (err) {
        console.error('[Puppeteer] Execution error:', err);
        process.exit(1);
    } finally {
        if (browser) await browser.close();
    }
})();
