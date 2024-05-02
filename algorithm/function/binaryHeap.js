// More info: https://eloquentjavascript.net/1st_edition/appendix2.html
class BinaryHeap {
    constructor(scoreFunction) {
        this.content = [];
        this.scoreFunction = scoreFunction;
    }

    sortElement(node) {
        this.bubbleUp(this.content.indexOf(node));
    }

    push(element) {
        // Add the new element to the end of the array.
        this.content.push(element);

        // Allow it to bubble up.
        this.bubbleUp(this.content.length - 1);
    }

    shift() {
        // Store the first element so we can return it later.
        const result = this.content[0];
        // Get the element at the end of the array.
        const end = this.content.pop();
        // If there are any elements left, put the end element at the
        // start, and let it sink down.
        if (this.content.length > 0) {
            this.content[0] = end;
            this.sinkDown(0);
        }
        return result;
    }

    remove(node) {
        const i = this.content.indexOf(node);

        // When it is found, the process seen in 'pop' is repeated
        // to fill up the hole.
        const end = this.content.pop();

        if (i !== this.content.length - 1) {
            this.content[i] = end;
            this.scoreFunction(end) < this.scoreFunction(node) ? this.bubbleUp(i) : this.sinkDown(i);
        }
    }

    size() {
        return this.content.length;
    }

    bubbleUp(n) {
        // Fetch the element that has to be moved.
        const element = this.content[n];

        // When at 0, an element can not go up any further.
        while (n > 0) {

            // Compute the parent element's index, and fetch it.
            const parentN = ((n + 1) >> 1) - 1;      // (n + 1) >> 1 equals Math.floor((n + 1) / 2)
            const parent = this.content[parentN];

            // Found a parent that is less, no need to sink any further.
            if (this.scoreFunction(element) >= this.scoreFunction(parent)) break;
            // Swap the elements if the parent is greater.
            this.content[parentN] = element;
            this.content[n] = parent;
            // Update 'n' to continue at the new position.
            n = parentN;
        }
    }
    sinkDown(n) {
        // Look up the target element and its score.
        const length = this.content.length;
        const element = this.content[n];
        const elemScore = this.scoreFunction(element);

        while (true) {
            // Compute the indices of the child elements.
            const child2N = (n + 1) << 1;
            const child1N = child2N - 1;
            // This is used to store the new position of the element, if any.
            let child1Score, child2Score, swap = null;
            // If the first child exists (is inside the array)...
            if (child1N < length) {
                // Look it up and compute its score.
                let child1 = this.content[child1N];
                child1Score = this.scoreFunction(child1);

                // If the score is less than our element's, we need to swap.
                if (child1Score < elemScore) swap = child1N;
            }

            // Do the same checks for the other child.
            if (child2N < length) {
                const child2 = this.content[child2N];
                child2Score = this.scoreFunction(child2);
                if (child2Score < (swap ? child1Score : elemScore)) swap = child2N;
            }

            if (!swap) break;
            // If the element needs to be moved, swap it, and continue.
            this.content[n] = this.content[swap];
            this.content[swap] = element;
            n = swap;
        }
    }
}

module.exports = { BinaryHeap };