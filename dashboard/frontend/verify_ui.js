import puppeteer from 'puppeteer';

(async () => {
    const outputPath = process.argv[2] || '/home/capstone/capstone_project_antigravity/results/dashboard_demo_polish_verification.png';
    console.log(`[Puppeteer] Starting UI verification. Capturing to ${outputPath}...`);
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
            if (msg.type === 'error') {
                errors.push(msg.text());
                console.error('[BROWSER_CONSOLE_ERROR]', msg.text());
            } else {
                console.log('[BROWSER_CONSOLE]', msg.text());
            }
        });

        page.on('pageerror', err => {
            errors.push(err.message);
            console.error('[PAGE_ERROR]', err.message);
        });

        console.log('[Puppeteer] Navigating to http://localhost:5173...');
        await page.goto('http://localhost:5173', { waitUntil: 'domcontentloaded', timeout: 15000 });

        // Wait 3 seconds for WebSocket and REST calls to populate
        await new Promise(r => setTimeout(r, 3000));

        // Evaluate text content on the page to verify:
        // 1. No "undefined active / undefined total"
        // 2. No "0.9008%"
        // 3. No "NaN%"
        const pageText = await page.evaluate(() => document.body.innerText);

        const hasUndefinedTasks = pageText.includes('undefined active') || pageText.includes('undefined total');
        const hasUnscaledCoverage = pageText.includes('0.9008%') || pageText.includes('0.30504%');
        const hasNanCoverage = pageText.includes('NaN%');

        console.log(`[CHECK] Has undefined tasks: ${hasUndefinedTasks ? 'FAIL (Found undefined)' : 'PASS (Clean)'}`);
        console.log(`[CHECK] Has unscaled coverage fraction: ${hasUnscaledCoverage ? 'FAIL (Found unscaled fraction)' : 'PASS (Clean)'}`);
        console.log(`[CHECK] Has NaN coverage: ${hasNanCoverage ? 'FAIL (Found NaN%)' : 'PASS (Clean)'}`);
        console.log(`[CHECK] Total console errors: ${errors.length}`);

        await page.screenshot({ path: outputPath, fullPage: true });
        console.log(`[Puppeteer] Screenshot saved successfully: ${outputPath}`);

        if (hasUndefinedTasks || hasUnscaledCoverage || hasNanCoverage || errors.length > 0) {
            console.error('[VERIFICATION] Failed verification checks!');
            process.exit(1);
        } else {
            console.log('[VERIFICATION] All dashboard UI verification checks PASSED!');
            process.exit(0);
        }
    } catch (err) {
        console.error('[Puppeteer] Execution error:', err);
        process.exit(1);
    } finally {
        if (browser) await browser.close();
    }
})();
