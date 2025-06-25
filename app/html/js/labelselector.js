function LabelSelector(elementId, ignoreLabels, callback) {
  if (typeof callback !== 'function') {
    console.error('Callback must be a function.');
    return;
  }

  const element = document.getElementById(elementId);
  if (!element) {
    console.error(`Element with ID ${elementId} not found.`);
    return;
  }

  // Define all possible labels
  const allLabels = [
    "Human", "Face", "Car", "Truck", "Bus", "Vehicle", "Bike", "Object"
  ];

  // Generate HTML for checkboxes
  let html = '';
  let columnIndex = 0;
  html += '<div class="row">';
  
  allLabels.forEach((label) => {
    if (columnIndex === 0) {
      html += '<div class="col-4">';
    }
    
	
    const isChecked = ignoreLabels.includes(label)?'':'checked';
    html += '<div class="form-check">'
	html += '<input class="form-check-input" type="checkbox" value="' + label + '" id="class-' + label + '" ' + isChecked + '>';
    html += '<label class="form-check-label" for="class-' + label + '">' + label + '</label>';
    html += '</div>';

    columnIndex++;
    if (columnIndex === 3) {
      html += '</div>';
      columnIndex = 0;
    }
  });

  // Close any remaining column divs
  if (columnIndex > 0) {
    html += '</div>';
  }
  
  html += '</div>';
  
  element.innerHTML = html;

  // Add change event handler for all checkboxes
  const checkboxes = element.querySelectorAll('.form-check-input');
  checkboxes.forEach((checkbox) => {
    checkbox.addEventListener('change', (event) => {
      updateSelection();
    });
  });

  function updateSelection() {
    const checkboxes = element.querySelectorAll('.form-check-input');
    const selectedLabels = [];

    checkboxes.forEach((checkbox) => {
      if (!checkbox.checked) {
        selectedLabels.push(checkbox.value);
      }
    });

    callback(selectedLabels);
  }
}

// To make LabelSelector available globally
window.LabelSelector = LabelSelector;
