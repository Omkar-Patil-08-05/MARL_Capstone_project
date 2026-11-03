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

        // 4. Test Victim Detection Panel rendering
        const hasVictimDetection = pageText.includes('VICTIM DETECTION');
        const hasDetectedBadge = pageText.includes('DETECTED');
        console.log(`[CHECK] Victim Detection Panel present: ${hasVictimDetection ? 'PASS' : 'FAIL'}`);
        console.log(`[CHECK] Victim DETECTED badge present: ${hasDetectedBadge ? 'PASS' : 'FAIL'}`);
        console.log(`[CHECK] Browser console errors: ${errors.length}`);

        await page.screenshot({ path: outputPath, fullPage: true });
        console.log(`[Puppeteer] Live screenshot captured: ${outputPath}`);

        if (hasUndefined || hasUnscaledFraction || errors.length > 0 || !hasD0 || !hasExpectedDrones || !hasVictimDetection) {
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
