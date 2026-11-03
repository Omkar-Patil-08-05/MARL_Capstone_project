import puppeteer from 'puppeteer';

(async () => {
    const outputPath = process.argv[2] || '/tmp/dashboard_six_drone_live.png';
    console.log(`[Puppeteer] Capturing dashboard to ${outputPath}...`);
    let browser;
    try {
        browser = await puppeteer.launch({
            executablePath: '/usr/bin/google-chrome-stable',
            args: ['--no-sandbox', '--disable-setuid-sandbox', '--disable-gpu', '--window-size=1920,1080']
        });
        const page = await browser.newPage();
        await page.setViewport({ width: 1920, height: 1080 });
        await page.goto('http://localhost:5173', { waitUntil: 'networkidle2', timeout: 30000 });
        
        // Wait 2 seconds for WebSocket telemetry rendering to stabilize
        await new Promise(r => setTimeout(r, 2000));
        
        await page.screenshot({ path: outputPath, fullPage: true });
        console.log(`[Puppeteer] Screenshot saved successfully: ${outputPath}`);
    } catch (err) {
        console.error(`[Puppeteer] Error capturing dashboard:`, err);
        process.exit(1);
    } finally {
        if (browser) {
            await browser.close();
        }
    }
})();
