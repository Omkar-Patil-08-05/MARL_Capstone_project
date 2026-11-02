import puppeteer from 'puppeteer';
import fs from 'fs';

(async () => {
    console.log("=== STARTING BROWSER PUPPETEER MAP INSPECTION ===");
    const browser = await puppeteer.launch({
        executablePath: '/usr/bin/google-chrome',
        args: ['--no-sandbox', '--disable-setuid-sandbox', '--disable-gpu', '--window-size=1920,1080']
    });

    try {
        const page = await browser.newPage();
        await page.setViewport({ width: 1920, height: 1080 });
        await page.goto('http://localhost:5173', { waitUntil: 'networkidle0' });

        // 1. Check if on Mission Setup screen, select Forest SAR if needed
        const forestOption = await page.$('text/Forest Wildland SAR');
        if (forestOption) {
            console.log("Found Forest Wildland SAR option. Clicking it...");
            await forestOption.click();
            await new Promise(r => setTimeout(r, 1000));
        }

        // Check if there is an active map or we can inspect map component
        // Let's check the DOM for SVG map elements
        const svgElement = await page.$('svg');
        console.log("SVG on page:", svgElement ? "Found" : "None on setup screen (expected before start or during active run)");

        await page.screenshot({ path: '/tmp/forest_setup_screen.png' });
        console.log("Saved /tmp/forest_setup_screen.png");

        console.log("=== BROWSER INITIAL INSPECTION SUCCESSFUL ===");
    } catch (e) {
        console.error("Puppeteer test error:", e);
    } finally {
        await browser.close();
    }
})();
